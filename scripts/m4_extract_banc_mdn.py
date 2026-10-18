"""MW-M4.1: Resolve BANC MDN identity and complete v888 subgraph extraction."""

import csv
import hashlib
import json
import pathlib
import re
import sys
import time
import urllib.error
import urllib.request

import numpy as np
import pyarrow.feather as feather
import pyarrow.ipc as ipc

# Dataset & source specifications
BANC_VERSION = "v888"
NATURE_DOI = "10.1038/s41586-026-10735-w"
DATAVERSE_DOI = "10.7910/DVN/7WTH1N"
SOURCE_LICENSE = "CC BY 4.0"
SPECIMEN_SEX = "female"
COVERAGE = "Whole brain and ventral nerve cord"

METADATA_URL = "https://storage.googleapis.com/lee-lab_brain-and-nerve-cord-fly-connectome/compiled_data/banc_888/banc_888_meta.feather"
EDGELIST_URL = "https://storage.googleapis.com/lee-lab_brain-and-nerve-cord-fly-connectome/compiled_data/banc_888/banc_888_edgelist_simple_v2.feather"

EXPECTED_METADATA_BYTES = 57503026
EXPECTED_METADATA_SHA256 = "86ccf5df0c67419f8c5f43e93a7ed38d23a080e9f7fde26737290252f3780098"

EXPECTED_EDGELIST_BYTES = 305250378
EXPECTED_EDGELIST_SHA256 = "363fdef3813b72a5e45a42f17034cd5a544b654c838929cecf6e1ce5f60625cb"

MANAGER_EXPECTED_MDN_IDS = {
    "720575941491012809",
    "720575941690515574",
    "720575941592674814",
    "720575941597827697",
}

RAW_DIR = pathlib.Path("data/raw/banc_v888")
DERIVED_DIR = pathlib.Path("data/derived")

META_PATH = RAW_DIR / "banc_888_meta.feather"
EDGELIST_PATH = RAW_DIR / "banc_888_edgelist_simple_v2.feather"

MANIFEST_PATH = DERIVED_DIR / "banc_v888_mdn_manifest.json"
EDGES_CSV_PATH = DERIVED_DIR / "banc_v888_mdn_edges.csv"

MIN_SYNAPSES = 3
MAX_FILE_BYTES = 2 * 1024 * 1024 * 1024  # 2 GiB safety limit


def compute_sha256(file_path: pathlib.Path) -> str:
    """Compute SHA-256 hash of a file."""
    h = hashlib.sha256()
    with open(file_path, "rb") as f:
        while chunk := f.read(1024 * 1024):
            h.update(chunk)
    return h.hexdigest()


def download_or_reuse_file(url: str, dest_path: pathlib.Path, expected_bytes: int | None = None, expected_sha256: str | None = None) -> tuple[int, str, bool]:
    """Verify and reuse existing file if valid, otherwise download safely."""
    dest_path.parent.mkdir(parents=True, exist_ok=True)
    reused = False

    if dest_path.exists() and dest_path.stat().st_size > 0:
        local_size = dest_path.stat().st_size
        if expected_bytes is None or local_size == expected_bytes:
            print(f"Verifying local file: {dest_path} ({local_size} bytes)...")
            sha256 = compute_sha256(dest_path)
            if expected_sha256 is None or sha256 == expected_sha256:
                print(f"Verified local file SHA-256: {sha256}")
                return local_size, sha256, True
            else:
                print(f"Local file SHA-256 mismatch ({sha256} != {expected_sha256}); redownload required.")

    # Download
    print(f"Downloading from: {url}")
    part_path = dest_path.with_name(dest_path.name + ".part")
    req = urllib.request.Request(
        url,
        headers={"User-Agent": "MoonwalkerFly-M4/1.0 (Connectome-Extraction; macOS)"},
    )

    with urllib.request.urlopen(req, timeout=30) as response:
        content_length_str = response.headers.get("Content-Length")
        remote_size = int(content_length_str) if content_length_str else None

    if remote_size is not None and remote_size > MAX_FILE_BYTES:
        raise RuntimeError(f"ABORT: Remote file size {remote_size} bytes exceeds safety limit!")

    h = hashlib.sha256()
    bytes_downloaded = 0
    t0 = time.time()
    last_print = t0

    with urllib.request.urlopen(req, timeout=60) as response, open(part_path, "wb") as out_f:
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            out_f.write(chunk)
            h.update(chunk)
            bytes_downloaded += len(chunk)

            now = time.time()
            if now - last_print >= 5.0:
                rate = bytes_downloaded / (now - t0) / (1024 * 1024)
                pct = f"{bytes_downloaded / remote_size * 100:.1f}%" if remote_size else "unknown"
                print(f"  Downloaded {bytes_downloaded / (1024*1024):.2f} MB ({pct}) at {rate:.2f} MB/s...")
                last_print = now

    part_path.replace(dest_path)
    sha256 = h.hexdigest()
    print(f"Saved to: {dest_path} ({bytes_downloaded} bytes, SHA-256: {sha256})")
    return bytes_downloaded, sha256, False


def is_mdn_text_match(text: str) -> bool:
    """Determine if a text annotation contains MDN / DNp50 / Moonwalker with word awareness."""
    if not isinstance(text, str) or not text.strip():
        return False
    lower = text.strip().lower()
    if "moonwalker" in lower:
        return True
    if "dnp50" in lower:
        return True
    tokens = re.split(r"[\s,;:/|()\[\]{}\-_\.]+", lower)
    for t in tokens:
        if t in ("mdn", "mdnr", "mdnl") or re.match(r"^mdn[0-9_lr]?$", t):
            return True
    return False


def find_column(available_cols: list[str], candidates: list[str]) -> str | None:
    """Find the first matching candidate in available column names (case-insensitive)."""
    col_lower = {c.lower(): c for c in available_cols}
    for cand in candidates:
        if cand.lower() in col_lower:
            return col_lower[cand.lower()]
    return None


def main() -> None:
    print("=== MW-M4.1: RESOLVE BANC MDN IDENTITY & SUBGRAPH EXTRACTION ===")
    print(f"BANC_VERSION={BANC_VERSION}")

    # -------------------------------------------------------------
    # 1. Verify and Reuse Existing Metadata
    # -------------------------------------------------------------
    print("\n--- STEP 1: VERIFYING EXISTING BANC v888 METADATA ---")
    meta_bytes, meta_sha256, meta_reused = download_or_reuse_file(
        METADATA_URL,
        META_PATH,
        expected_bytes=EXPECTED_METADATA_BYTES,
        expected_sha256=EXPECTED_METADATA_SHA256,
    )
    print(f"METADATA_BYTES={meta_bytes}")
    print(f"METADATA_SHA256={meta_sha256}")
    print(f"METADATA_REUSED_FROM_M4={'YES' if meta_reused else 'NO'}")

    meta_table = feather.read_table(META_PATH)
    meta_rows = meta_table.num_rows
    meta_cols = meta_table.num_columns
    meta_col_names = meta_table.column_names

    print(f"METADATA_ROWS={meta_rows}")
    print(f"METADATA_COLUMNS={meta_cols}")

    root_id_col = find_column(meta_col_names, ["root_id"])
    banc_id_col = find_column(meta_col_names, ["banc_888_id"])
    assert root_id_col and banc_id_col, "Missing root_id or banc_888_id column!"
    print(f"ROOT_ID_COLUMN={root_id_col}")
    print(f"BANC_ID_COLUMN={banc_id_col}")

    cell_type_col = find_column(meta_col_names, ["cell_type"])
    side_col = find_column(meta_col_names, ["side"])
    superclass_col = find_column(meta_col_names, ["super_class"])
    cell_class_col = find_column(meta_col_names, ["cell_class"])
    cell_subclass_col = find_column(meta_col_names, ["cell_sub_class"])
    flow_col = find_column(meta_col_names, ["flow"])
    nt_col = find_column(meta_col_names, ["neurotransmitter_predicted"])
    nt_score_col = find_column(meta_col_names, ["neurotransmitter_score"])
    hemilineage_col = find_column(meta_col_names, ["hemilineage"])
    neuromere_col = find_column(meta_col_names, ["neuromere"])
    function_col = find_column(meta_col_names, ["cell_function"])
    function_detailed_col = find_column(meta_col_names, ["cell_function_detailed"])

    # Cross-dataset columns
    fafb_type_col = find_column(meta_col_names, ["fafb_cell_type"])
    manc_type_col = find_column(meta_col_names, ["manc_cell_type"])
    malecns_type_col = find_column(meta_col_names, ["malecns_cell_type"])
    hemibrain_type_col = find_column(meta_col_names, ["hemibrain_cell_type"])
    fanc_type_col = find_column(meta_col_names, ["fanc_cell_type"])

    root_ids_raw = [str(r) for r in meta_table[root_id_col].to_pylist()]
    banc_ids_raw = [str(r) for r in meta_table[banc_id_col].to_pylist()]

    # -------------------------------------------------------------
    # 2. Canonical MDN Identity Extraction (Primary BANC Rule)
    # -------------------------------------------------------------
    print("\n--- STEP 2: CANONICAL MDN IDENTITY EXTRACTION ---")
    print("CANONICAL_IDENTITY_RULE: primary BANC cell_type == 'MDN'")

    canonical_mdn_indices = []
    cell_types_raw = meta_table[cell_type_col].to_pylist()
    for idx, ct in enumerate(cell_types_raw):
        if ct is not None and str(ct).strip() == "MDN":
            canonical_mdn_indices.append(idx)

    print(f"Rows matching primary BANC cell_type == 'MDN': {len(canonical_mdn_indices)}")

    canonical_mdn_records = []
    canonical_root_ids = set()
    canonical_banc_ids = set()
    left_count = 0
    right_count = 0

    mdn_by_banc_id = {}
    mdn_by_root_id = {}

    for idx in canonical_mdn_indices:
        r_id = root_ids_raw[idx]
        b_id = banc_ids_raw[idx]
        canonical_root_ids.add(r_id)
        canonical_banc_ids.add(b_id)

        ct = str(meta_table[cell_type_col][idx].as_py()) if cell_type_col and meta_table[cell_type_col][idx].is_valid else None
        side = str(meta_table[side_col][idx].as_py()) if side_col and meta_table[side_col][idx].is_valid else None
        s_class = str(meta_table[superclass_col][idx].as_py()) if superclass_col and meta_table[superclass_col][idx].is_valid else None
        c_class = str(meta_table[cell_class_col][idx].as_py()) if cell_class_col and meta_table[cell_class_col][idx].is_valid else None
        c_subclass = str(meta_table[cell_subclass_col][idx].as_py()) if cell_subclass_col and meta_table[cell_subclass_col][idx].is_valid else None
        flow = str(meta_table[flow_col][idx].as_py()) if flow_col and meta_table[flow_col][idx].is_valid else None
        nt = str(meta_table[nt_col][idx].as_py()) if nt_col and meta_table[nt_col][idx].is_valid else None
        nt_score = float(meta_table[nt_score_col][idx].as_py()) if nt_score_col and meta_table[nt_score_col][idx].is_valid else None
        hemi = str(meta_table[hemilineage_col][idx].as_py()) if hemilineage_col and meta_table[hemilineage_col][idx].is_valid else None
        neuromere = str(meta_table[neuromere_col][idx].as_py()) if neuromere_col and meta_table[neuromere_col][idx].is_valid else None
        c_func = str(meta_table[function_col][idx].as_py()) if function_col and meta_table[function_col][idx].is_valid else None
        c_func_det = str(meta_table[function_detailed_col][idx].as_py()) if function_detailed_col and meta_table[function_detailed_col][idx].is_valid else None

        supporting_cross_dataset = {
            "fafb_cell_type": str(meta_table[fafb_type_col][idx].as_py()) if fafb_type_col and meta_table[fafb_type_col][idx].is_valid else None,
            "manc_cell_type": str(meta_table[manc_type_col][idx].as_py()) if manc_type_col and meta_table[manc_type_col][idx].is_valid else None,
            "malecns_cell_type": str(meta_table[malecns_type_col][idx].as_py()) if malecns_type_col and meta_table[malecns_type_col][idx].is_valid else None,
            "hemibrain_cell_type": str(meta_table[hemibrain_type_col][idx].as_py()) if hemibrain_type_col and meta_table[hemibrain_type_col][idx].is_valid else None,
            "fanc_cell_type": str(meta_table[fanc_type_col][idx].as_py()) if fanc_type_col and meta_table[fanc_type_col][idx].is_valid else None,
        }

        if side == "left":
            left_count += 1
        elif side == "right":
            right_count += 1

        rec = {
            "root_id": r_id,
            "banc_888_id": b_id,
            "cell_type": ct,
            "side": side,
            "super_class": s_class,
            "cell_class": c_class,
            "cell_sub_class": c_subclass,
            "flow": flow,
            "neurotransmitter_predicted": nt,
            "neurotransmitter_score": nt_score,
            "hemilineage": hemi,
            "neuromere": neuromere,
            "cell_function": c_func,
            "cell_function_detailed": c_func_det,
            "supporting_cross_dataset_annotations": supporting_cross_dataset,
        }
        canonical_mdn_records.append(rec)
        mdn_by_banc_id[b_id] = rec
        mdn_by_root_id[r_id] = rec

    print(f"PRIMARY_BANC_MDN_COUNT={len(canonical_root_ids)}")
    print(f"PRIMARY_BANC_MDN_LEFT_COUNT={left_count}")
    print(f"PRIMARY_BANC_MDN_RIGHT_COUNT={right_count}")

    # Consistency checks on canonical records
    assert len(canonical_root_ids) == 4, f"Expected exactly 4 canonical MDNs, found {len(canonical_root_ids)}"
    assert left_count == 2, f"Expected 2 left MDNs, found {left_count}"
    assert right_count == 2, f"Expected 2 right MDNs, found {right_count}"
    for rec in canonical_mdn_records:
        assert rec["super_class"] == "descending", f"Expected super_class descending, got {rec['super_class']}"
        assert rec["cell_class"] == "descending_neuron", f"Expected cell_class descending_neuron, got {rec['cell_class']}"

    # Manager Expected ID Set Cross-check
    id_set_match = canonical_root_ids == MANAGER_EXPECTED_MDN_IDS
    print(f"MANAGER_EXPECTED_ID_SET_MATCH={'YES' if id_set_match else 'NO'}")
    if not id_set_match:
        raise RuntimeError(f"Canonical root IDs {canonical_root_ids} != expected set {MANAGER_EXPECTED_MDN_IDS}!")

    # -------------------------------------------------------------
    # 3. Identify Noncanonical Cross-Dataset Matches
    # -------------------------------------------------------------
    print("\n--- STEP 3: RECORDING NONCANONICAL CROSS-DATASET MATCHES ---")
    text_cols_to_search = []
    for col_name in meta_col_names:
        field = meta_table.schema.field(col_name)
        type_str = str(field.type)
        if "string" in type_str or "large_string" in type_str or "dictionary" in type_str:
            text_cols_to_search.append(col_name)

    noncanonical_matches = []
    for row_idx, r_id in enumerate(root_ids_raw):
        if r_id in canonical_root_ids:
            continue
        matching_fields = {}
        for col in text_cols_to_search:
            val = meta_table[col][row_idx].as_py()
            if val is not None and is_mdn_text_match(str(val)):
                matching_fields[col] = str(val)
        if matching_fields:
            ct = str(meta_table[cell_type_col][row_idx].as_py()) if cell_type_col and meta_table[cell_type_col][row_idx].is_valid else None
            side = str(meta_table[side_col][row_idx].as_py()) if side_col and meta_table[side_col][row_idx].is_valid else None
            s_class = str(meta_table[superclass_col][row_idx].as_py()) if superclass_col and meta_table[superclass_col][row_idx].is_valid else None
            noncanonical_matches.append({
                "root_id": r_id,
                "banc_888_id": banc_ids_raw[row_idx],
                "primary_cell_type": ct,
                "side": side,
                "super_class": s_class,
                "matching_imported_fields": matching_fields,
            })

    print(f"Discovered {len(noncanonical_matches)} noncanonical MDN-like text match(es):")
    for nm in noncanonical_matches:
        print(f"  Root {nm['root_id']}: primary_cell_type={nm['primary_cell_type']}, super_class={nm['super_class']}, fields={nm['matching_imported_fields']}")

    # -------------------------------------------------------------
    # 4. Build Partner Metadata Lookup by banc_888_id
    # -------------------------------------------------------------
    print("\n--- STEP 4: INDEXING METADATA FOR PARTNER ANNOTATIONS ---")
    # All edges in edgelist_simple_v2 use banc_888_id
    partner_meta_lookup = {}
    for idx, b_id in enumerate(banc_ids_raw):
        partner_meta_lookup[b_id] = {
            "root_id": root_ids_raw[idx],
            "banc_888_id": b_id,
            "cell_type": str(meta_table[cell_type_col][idx].as_py()) if cell_type_col and meta_table[cell_type_col][idx].is_valid else None,
            "side": str(meta_table[side_col][idx].as_py()) if side_col and meta_table[side_col][idx].is_valid else None,
            "super_class": str(meta_table[superclass_col][idx].as_py()) if superclass_col and meta_table[superclass_col][idx].is_valid else None,
            "cell_class": str(meta_table[cell_class_col][idx].as_py()) if cell_class_col and meta_table[cell_class_col][idx].is_valid else None,
            "cell_subclass": str(meta_table[cell_subclass_col][idx].as_py()) if cell_subclass_col and meta_table[cell_subclass_col][idx].is_valid else None,
            "nt_type": str(meta_table[nt_col][idx].as_py()) if nt_col and meta_table[nt_col][idx].is_valid else None,
        }
    print(f"Indexed {len(partner_meta_lookup)} neurons in partner metadata lookup.")

    del meta_table
    del root_ids_raw
    del banc_ids_raw

    # -------------------------------------------------------------
    # 5. Acquire / Verify Edge List
    # -------------------------------------------------------------
    print("\n--- STEP 5: VERIFYING / ACQUIRING BANC v888 SIMPLE v2 EDGE LIST ---")
    edge_bytes, edge_sha256, edge_reused = download_or_reuse_file(
        EDGELIST_URL,
        EDGELIST_PATH,
        expected_bytes=EXPECTED_EDGELIST_BYTES,
        expected_sha256=EXPECTED_EDGELIST_SHA256,
    )
    print(f"EDGELIST_BYTES={edge_bytes}")
    print(f"EDGELIST_SHA256={edge_sha256}")
    print(f"EDGELIST_DOWNLOADED_THIS_TASK={'NO' if edge_reused else 'YES'}")

    with open(EDGELIST_PATH, "rb") as f:
        reader = ipc.RecordBatchFileReader(f)
        edge_schema = reader.schema
        edge_col_names = edge_schema.names
        print(f"Edge list schema column names: {edge_col_names}")

    pre_col = find_column(edge_col_names, ["pre"])
    post_col = find_column(edge_col_names, ["post"])
    weight_col = find_column(edge_col_names, ["count"])

    assert pre_col and post_col and weight_col, "Cannot identify pre/post/count columns!"
    print(f"PRE_ID_COLUMN={pre_col}")
    print(f"POST_ID_COLUMN={post_col}")
    print(f"SYNAPSE_COUNT_COLUMN={weight_col}")

    # -------------------------------------------------------------
    # 6. Read Edge List Columns & Check for Duplicate Ordered Pairs
    # -------------------------------------------------------------
    print("\n--- STEP 6: SELECTIVE READING & DUPLICATE ORDERED PAIR CHECK ---")
    t0 = time.time()
    edge_table = feather.read_table(EDGELIST_PATH, columns=[pre_col, post_col, weight_col])
    edge_rows = edge_table.num_rows
    print(f"EDGE_LIST_ROW_COUNT={edge_rows} (read in {time.time() - t0:.2f} s)")

    pre_arr = edge_table[pre_col].to_numpy()
    post_arr = edge_table[post_col].to_numpy()
    weight_arr = edge_table[weight_col].to_numpy()

    print("Checking whether the simple edge list contains duplicate ordered (pre, post) pairs...")
    t_dup0 = time.time()
    sort_order = np.lexsort((post_arr, pre_arr))
    adj_pre_same = pre_arr[sort_order[1:]] == pre_arr[sort_order[:-1]]
    adj_post_same = post_arr[sort_order[1:]] == post_arr[sort_order[:-1]]
    dup_mask = adj_pre_same & adj_post_same
    dup_count = int(np.count_nonzero(dup_mask))
    print(f"Duplicate ordered pairs in full edgelist: {dup_count} (checked in {time.time() - t_dup0:.2f} s)")
    duplicate_ordered_pairs_found = dup_count > 0
    print(f"DUPLICATE_ORDERED_PAIRS_FOUND={'YES' if duplicate_ordered_pairs_found else 'NO'}")
    if duplicate_ordered_pairs_found:
        raise RuntimeError(f"Unexpected duplicate ordered pairs detected in simple edgelist: {dup_count}")

    del sort_order, adj_pre_same, adj_post_same, dup_mask

    # -------------------------------------------------------------
    # 7. Extract Direct Connectivity for Canonical MDNs
    # -------------------------------------------------------------
    print("\n--- STEP 7: EXTRACTING DIRECT CANONICAL MDN CONNECTIVITY ---")
    # Query edgelist using canonical banc_888_ids
    if pre_arr.dtype.kind in ("U", "S", "O"):
        canonical_arr = np.array(list(canonical_banc_ids), dtype=pre_arr.dtype)
    else:
        canonical_arr = np.array([int(x) for x in canonical_banc_ids], dtype=pre_arr.dtype)

    pre_is_mdn = np.isin(pre_arr, canonical_arr)
    post_is_mdn = np.isin(post_arr, canonical_arr)
    weight_ge_min = weight_arr >= MIN_SYNAPSES

    retained_mask = (pre_is_mdn | post_is_mdn) & weight_ge_min
    retained_indices = np.where(retained_mask)[0]
    print(f"Total raw matching edges touching canonical MDNs (MIN_SYNAPSES >= {MIN_SYNAPSES}): {len(retained_indices)}")

    derived_rows = []
    canonical_banc_id_str_set = {str(x) for x in canonical_banc_ids}

    for idx in retained_indices:
        pre_b_id = str(pre_arr[idx])
        post_b_id = str(post_arr[idx])
        syn_cnt = int(weight_arr[idx])

        # If pre is canonical MDN: DOWNSTREAM connection
        if pre_b_id in canonical_banc_id_str_set:
            mdn_info = mdn_by_banc_id[pre_b_id]
            partner_b_id = post_b_id
            partner_info = partner_meta_lookup[partner_b_id]
            derived_rows.append({
                "mdn_root_id": mdn_info["root_id"],
                "mdn_banc_888_id": pre_b_id,
                "direction_relative_to_mdn": "DOWNSTREAM",
                "partner_root_id": partner_info["root_id"],
                "partner_banc_888_id": partner_b_id,
                "pre_root_id": mdn_info["root_id"],
                "post_root_id": partner_info["root_id"],
                "pre_banc_888_id": pre_b_id,
                "post_banc_888_id": post_b_id,
                "synapse_count": syn_cnt,
                "partner_cell_type": partner_info.get("cell_type"),
                "partner_side": partner_info.get("side"),
                "partner_super_class": partner_info.get("super_class"),
                "partner_class": partner_info.get("cell_class"),
                "partner_subclass": partner_info.get("cell_subclass"),
                "partner_nt_type": partner_info.get("nt_type"),
            })

        # If post is canonical MDN: UPSTREAM connection
        if post_b_id in canonical_banc_id_str_set:
            mdn_info = mdn_by_banc_id[post_b_id]
            partner_b_id = pre_b_id
            partner_info = partner_meta_lookup[partner_b_id]
            derived_rows.append({
                "mdn_root_id": mdn_info["root_id"],
                "mdn_banc_888_id": post_b_id,
                "direction_relative_to_mdn": "UPSTREAM",
                "partner_root_id": partner_info["root_id"],
                "partner_banc_888_id": partner_b_id,
                "pre_root_id": partner_info["root_id"],
                "post_root_id": mdn_info["root_id"],
                "pre_banc_888_id": partner_b_id,
                "post_banc_888_id": post_b_id,
                "synapse_count": syn_cnt,
                "partner_cell_type": partner_info.get("cell_type"),
                "partner_side": partner_info.get("side"),
                "partner_super_class": partner_info.get("super_class"),
                "partner_class": partner_info.get("cell_class"),
                "partner_subclass": partner_info.get("cell_subclass"),
                "partner_nt_type": partner_info.get("nt_type"),
            })

    del pre_arr, post_arr, weight_arr, edge_table

    # Deterministic sort:
    # 1. mdn_root_id (ascending)
    # 2. direction_relative_to_mdn (alphabetical: DOWNSTREAM, UPSTREAM)
    # 3. descending synapse_count
    # 4. partner_root_id (ascending)
    derived_rows.sort(
        key=lambda r: (
            r["mdn_root_id"],
            r["direction_relative_to_mdn"],
            -r["synapse_count"],
            r["partner_root_id"],
        )
    )

    retained_mdn_edge_count = len(derived_rows)
    total_upstream_edges = sum(1 for r in derived_rows if r["direction_relative_to_mdn"] == "UPSTREAM")
    total_downstream_edges = sum(1 for r in derived_rows if r["direction_relative_to_mdn"] == "DOWNSTREAM")
    total_upstream_synapses = sum(r["synapse_count"] for r in derived_rows if r["direction_relative_to_mdn"] == "UPSTREAM")
    total_downstream_synapses = sum(r["synapse_count"] for r in derived_rows if r["direction_relative_to_mdn"] == "DOWNSTREAM")

    print(f"RETAINED_MDN_EDGE_COUNT={retained_mdn_edge_count}")
    print(f"TOTAL_UPSTREAM_EDGES={total_upstream_edges}")
    print(f"TOTAL_DOWNSTREAM_EDGES={total_downstream_edges}")
    print(f"TOTAL_UPSTREAM_RETAINED_SYNAPSES={total_upstream_synapses}")
    print(f"TOTAL_DOWNSTREAM_RETAINED_SYNAPSES={total_downstream_synapses}")

    # -------------------------------------------------------------
    # 8. Consistency Verification
    # -------------------------------------------------------------
    print("\n--- STEP 8: CONSISTENCY VERIFICATION ---")
    assert len(canonical_root_ids) == 4, "Must have 4 canonical MDNs"
    assert left_count == 2 and right_count == 2, "Must be 2 left and 2 right"
    assert id_set_match, "Must match expected ID set"

    noncanonical_root_ids = {nm["root_id"] for nm in noncanonical_matches}
    for r in derived_rows:
        assert r["mdn_root_id"] not in noncanonical_root_ids, f"Noncanonical ID entered as MDN: {r['mdn_root_id']}"
        assert r["mdn_root_id"] in MANAGER_EXPECTED_MDN_IDS, f"Unknown MDN ID: {r['mdn_root_id']}"
        assert r["synapse_count"] >= MIN_SYNAPSES, f"Synapse count < {MIN_SYNAPSES}: {r}"
        if r["direction_relative_to_mdn"] == "UPSTREAM":
            assert r["post_root_id"] == r["mdn_root_id"]
            assert r["pre_root_id"] == r["partner_root_id"]
        elif r["direction_relative_to_mdn"] == "DOWNSTREAM":
            assert r["pre_root_id"] == r["mdn_root_id"]
            assert r["post_root_id"] == r["partner_root_id"]
        assert r["partner_banc_888_id"] in partner_meta_lookup, f"Partner not found in lookup: {r['partner_banc_888_id']}"

    print("All consistency checks PASSED!")

    # -------------------------------------------------------------
    # 9. Write Derived CSV
    # -------------------------------------------------------------
    print("\n--- STEP 9: WRITING DERIVED CSV ---")
    DERIVED_DIR.mkdir(parents=True, exist_ok=True)

    csv_fieldnames = [
        "mdn_root_id",
        "mdn_banc_888_id",
        "direction_relative_to_mdn",
        "partner_root_id",
        "partner_banc_888_id",
        "pre_root_id",
        "post_root_id",
        "pre_banc_888_id",
        "post_banc_888_id",
        "synapse_count",
        "partner_cell_type",
        "partner_side",
        "partner_super_class",
        "partner_class",
        "partner_subclass",
        "partner_nt_type",
    ]

    with open(EDGES_CSV_PATH, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=csv_fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(derived_rows)

    edge_csv_size = EDGES_CSV_PATH.stat().st_size
    print(f"Saved derived edges CSV: {EDGES_CSV_PATH} ({edge_csv_size} bytes, {edge_csv_size / (1024*1024):.2f} MB)")

    # -------------------------------------------------------------
    # 10. Build Manifest JSON
    # -------------------------------------------------------------
    print("\n--- STEP 10: WRITING DERIVED MANIFEST JSON ---")
    connectivity_summary = {}
    top_partners = {}

    sorted_canonical_root_ids = sorted(list(MANAGER_EXPECTED_MDN_IDS))

    for r_id in sorted_canonical_root_ids:
        up_edges = [r for r in derived_rows if r["mdn_root_id"] == r_id and r["direction_relative_to_mdn"] == "UPSTREAM"]
        down_edges = [r for r in derived_rows if r["mdn_root_id"] == r_id and r["direction_relative_to_mdn"] == "DOWNSTREAM"]

        connectivity_summary[r_id] = {
            "upstream_edge_count": len(up_edges),
            "downstream_edge_count": len(down_edges),
            "upstream_total_retained_synapses": sum(r["synapse_count"] for r in up_edges),
            "downstream_total_retained_synapses": sum(r["synapse_count"] for r in down_edges),
        }

        top_up = [
            {
                "partner_root_id": r["partner_root_id"],
                "partner_banc_888_id": r["partner_banc_888_id"],
                "partner_cell_type": r["partner_cell_type"],
                "synapse_count": r["synapse_count"],
            }
            for r in up_edges[:20]
        ]
        top_down = [
            {
                "partner_root_id": r["partner_root_id"],
                "partner_banc_888_id": r["partner_banc_888_id"],
                "partner_cell_type": r["partner_cell_type"],
                "synapse_count": r["synapse_count"],
            }
            for r in down_edges[:20]
        ]

        top_partners[r_id] = {
            "top_upstream_partners": top_up,
            "top_downstream_partners": top_down,
        }

    manifest_data = {
        "dataset": {
            "name": "BANC",
            "version": BANC_VERSION,
            "specimen_sex": SPECIMEN_SEX,
            "coverage": COVERAGE,
            "nature_doi": NATURE_DOI,
            "harvard_dataverse_doi": DATAVERSE_DOI,
            "source_license": SOURCE_LICENSE,
        },
        "provenance_and_rules": {
            "canonical_identity_rule": "primary BANC cell_type == 'MDN'",
            "identity_consistency_checks": "super_class == 'descending', cell_class == 'descending_neuron', exactly 2 left and 2 right",
            "cross_dataset_annotations": "supporting only; does not independently confer MDN membership",
            "cross_dataset_mdn_like_noncanonical_matches": noncanonical_matches,
        },
        "raw_sources": {
            "metadata_url": METADATA_URL,
            "metadata_bytes": meta_bytes,
            "metadata_sha256": meta_sha256,
            "metadata_reused_from_m4": meta_reused,
            "edge_url": EDGELIST_URL,
            "edge_bytes": edge_bytes,
            "edge_sha256": edge_sha256,
            "edge_downloaded_this_task": not edge_reused,
        },
        "extraction": {
            "min_synapses": MIN_SYNAPSES,
            "metadata_row_count": meta_rows,
            "edge_list_row_count": edge_rows,
            "duplicate_ordered_pairs_in_edgelist": duplicate_ordered_pairs_found,
            "retained_mdn_edge_count": retained_mdn_edge_count,
            "total_upstream_edges": total_upstream_edges,
            "total_downstream_edges": total_downstream_edges,
            "total_upstream_retained_synapses": total_upstream_synapses,
            "total_downstream_retained_synapses": total_downstream_synapses,
        },
        "mdn_neurons": canonical_mdn_records,
        "connectivity_summary": connectivity_summary,
        "top_partners": top_partners,
    }

    with open(MANIFEST_PATH, "w", encoding="utf-8") as f:
        json.dump(manifest_data, f, indent=2)

    manifest_size = MANIFEST_PATH.stat().st_size
    print(f"Saved manifest JSON: {MANIFEST_PATH} ({manifest_size} bytes, {manifest_size / 1024:.2f} KB)")

    # Print top 5 partners for console review
    print("\n=== STRONGEST DIRECT PARTNERS (TOP 5 PER MDN) ===")
    for r_id in sorted_canonical_root_ids:
        print(f"\nMDN {r_id}:")
        print("  TOP 5 UPSTREAM:")
        for p in top_partners[r_id]["top_upstream_partners"][:5]:
            print(f"    Partner {p['partner_root_id']} | Type: {p['partner_cell_type']} | Synapses: {p['synapse_count']}")
        print("  TOP 5 DOWNSTREAM:")
        for p in top_partners[r_id]["top_downstream_partners"][:5]:
            print(f"    Partner {p['partner_root_id']} | Type: {p['partner_cell_type']} | Synapses: {p['synapse_count']}")

    print("\n=== EXECUTION SUMMARY MARKERS ===")
    print(f"MANIFEST_PATH={MANIFEST_PATH.as_posix()}")
    print(f"MANIFEST_BYTES={manifest_size}")
    print(f"EDGE_CSV_PATH={EDGES_CSV_PATH.as_posix()}")
    print(f"EDGE_CSV_BYTES={edge_csv_size}")
    print("M4_EXTRACTION_RESULT=PASS")


if __name__ == "__main__":
    main()
