/**
 * Moonwalker Fly — Interactive Web Application Controller
 *
 * Manages responsive viewport layout, idle fly animations, theatrical
 * stage transitions, audio playback, and user interaction.
 */

document.addEventListener('DOMContentLoaded', () => {
  const videoIdle = document.getElementById('video-idle');
  const videoStage = document.getElementById('video-stage');
  const audioFunk = document.getElementById('audio-funk');

  const btnMoonwalk = document.getElementById('btn-moonwalk');
  const moonwalkLabel = document.getElementById('moonwalk-label');
  const btnReplay = document.getElementById('btn-replay');
  const replayLabel = document.getElementById('replay-label');

  // Presentation playback speed is fixed to 0.5x
  const PLAYBACK_SPEED = 0.5;

  const ASSETS = {
    landscape: {
      idles: [
        'assets/video_idle_16x9.mp4',
        'assets/video_idle_var2_16x9.mp4',
        'assets/video_idle_var3_16x9.mp4'
      ],
      stage: 'assets/video_stage_16x9.mp4',
      posterHabitat: 'assets/poster_habitat_16x9.png',
      posterStage: 'assets/poster_stage_16x9.png'
    },
    portrait: {
      idles: [
        'assets/video_idle_portrait.mp4',
        'assets/video_idle_var2_portrait.mp4',
        'assets/video_idle_var3_portrait.mp4'
      ],
      stage: 'assets/video_stage_portrait.mp4',
      posterHabitat: 'assets/poster_habitat_portrait.png',
      posterStage: 'assets/poster_stage_portrait.png'
    }
  };

  /**
   * Returns true if current viewport is portrait or taller than wide.
   */
  function isPortrait() {
    return window.matchMedia('(orientation: portrait), (max-aspect-ratio: 1/1)').matches;
  }

  let currentMode = isPortrait() ? 'portrait' : 'landscape';
  let currentIdleIdx = 0;
  let isPerforming = false;
  let audioFadeInterval = null;

  /**
   * Applies assets based on current orientation.
   */
  function applyOrientationAssets(force = false) {
    const newMode = isPortrait() ? 'portrait' : 'landscape';
    if (!force && newMode === currentMode) return;
    currentMode = newMode;

    const manifest = ASSETS[currentMode];

    // Update idle video
    const currentIdleTime = videoIdle.currentTime;
    const isIdlePlaying = !videoIdle.paused;
    videoIdle.poster = manifest.posterHabitat;
    videoIdle.src = manifest.idles[currentIdleIdx];
    if (currentIdleTime > 0) {
      videoIdle.currentTime = currentIdleTime;
    }
    if (isIdlePlaying || !isPerforming) {
      videoIdle.play().catch(e => console.warn('Idle autoplay deferred:', e));
    }

    // Update stage video
    videoStage.poster = manifest.posterStage;
    videoStage.src = manifest.stage;
    videoStage.playbackRate = PLAYBACK_SPEED;
  }

  // Initial asset application
  applyOrientationAssets(true);

  // Listen for media orientation query changes
  const mql = window.matchMedia('(orientation: portrait), (max-aspect-ratio: 1/1)');
  if (mql.addEventListener) {
    mql.addEventListener('change', () => applyOrientationAssets(false));
  } else if (mql.addListener) {
    mql.addListener(() => applyOrientationAssets(false));
  }
  window.addEventListener('resize', () => applyOrientationAssets(false));

  /**
   * Picks the next idle index randomly from the remaining candidates
   * to guarantee non-consecutive, varied, non-repetitive activity.
   */
  function getNextIdleIndex() {
    const total = ASSETS[currentMode].idles.length;
    const candidates = [];
    for (let i = 0; i < total; i++) {
      if (i !== currentIdleIdx) {
        candidates.push(i);
      }
    }
    const randChoice = Math.floor(Math.random() * candidates.length);
    return candidates[randChoice];
  }

  // Cycle through compatible idle variations on each loop completion
  videoIdle.addEventListener('ended', () => {
    if (isPerforming) return;
    currentIdleIdx = getNextIdleIndex();
    const manifest = ASSETS[currentMode];
    videoIdle.src = manifest.idles[currentIdleIdx];
    videoIdle.play().catch(e => console.warn('Error cycling idle variation:', e));
  });

  // Pre-configure stage video
  videoStage.playbackRate = PLAYBACK_SPEED;

  /**
   * Starts soundtrack playback, handling browser autoplay policies gracefully.
   */
  function playAudioSafely() {
    if (!audioFunk) return;
    if (audioFadeInterval) {
      clearInterval(audioFadeInterval);
      audioFadeInterval = null;
    }
    try {
      audioFunk.currentTime = 0;
      audioFunk.volume = 1.0;
      audioFunk.playbackRate = 1.0;
      const playPromise = audioFunk.play();
      if (playPromise !== undefined) {
        playPromise.catch(err => {
          console.warn('Audio playback prevented by browser policy:', err);
        });
      }
    } catch (err) {
      console.warn('Audio playback exception:', err);
    }
  }

  /**
   * Fades out audio over 150 ms before stopping.
   */
  function stopAudioSafely() {
    if (!audioFunk) return;
    if (audioFadeInterval) {
      clearInterval(audioFadeInterval);
      audioFadeInterval = null;
    }
    const fadeSteps = 10;
    const fadeIntervalMs = 15;
    let currentStep = 0;

    audioFadeInterval = setInterval(() => {
      currentStep++;
      const nextVolume = Math.max(0, 1.0 - (currentStep / fadeSteps));
      try {
        audioFunk.volume = nextVolume;
      } catch (e) {}

      if (currentStep >= fadeSteps || nextVolume <= 0) {
        clearInterval(audioFadeInterval);
        audioFadeInterval = null;
        try {
          audioFunk.pause();
          audioFunk.currentTime = 0;
          audioFunk.volume = 1.0;
        } catch (e) {}
      }
    }, fadeIntervalMs);
  }

  /**
   * Triggers the moonwalk stage performance.
   */
  async function startPerformance(isReplay = false) {
    if (isPerforming) return;
    isPerforming = true;

    if (!isReplay) {
      btnMoonwalk.classList.remove('is-hidden');
      btnReplay.classList.add('is-hidden');
      btnMoonwalk.disabled = true;
      btnMoonwalk.classList.add('active-walking');
      moonwalkLabel.textContent = 'Moonwalking…';
    } else {
      btnMoonwalk.classList.add('is-hidden');
      btnReplay.classList.remove('is-hidden');
      btnReplay.disabled = true;
      btnReplay.classList.add('active-walking');
      replayLabel.textContent = 'Replaying…';
    }

    playAudioSafely();

    const manifest = ASSETS[currentMode];
    if (!videoStage.src.endsWith(manifest.stage)) {
      videoStage.src = manifest.stage;
    }
    videoStage.currentTime = 0;
    videoStage.playbackRate = PLAYBACK_SPEED;

    try {
      await videoStage.play();

      videoStage.classList.remove('hidden');
      videoStage.classList.add('active');
      videoIdle.classList.remove('active');
      videoIdle.classList.add('hidden');
      videoIdle.pause();
    } catch (err) {
      console.error('Failed to play theatrical stage video:', err);
      returnToHabitat();
    }
  }

  /**
   * Returns to the natural habitat after performance completion.
   */
  function returnToHabitat() {
    isPerforming = false;

    stopAudioSafely();

    videoStage.pause();
    videoStage.currentTime = 0;

    videoIdle.classList.remove('hidden');
    videoIdle.classList.add('active');
    videoStage.classList.remove('active');
    videoStage.classList.add('hidden');

    videoIdle.play().catch(e => console.warn('Error resuming idle video:', e));

    btnMoonwalk.classList.add('is-hidden');
    btnMoonwalk.classList.remove('active-walking');
    moonwalkLabel.textContent = 'Moonwalk';
    btnMoonwalk.disabled = false;

    btnReplay.classList.remove('is-hidden');
    btnReplay.classList.remove('active-walking');
    replayLabel.textContent = 'Replay';
    btnReplay.disabled = false;
  }

  // Event Listeners
  btnMoonwalk.addEventListener('click', () => startPerformance(false));
  btnReplay.addEventListener('click', () => startPerformance(true));

  videoStage.addEventListener('ended', () => {
    if (isPerforming) {
      returnToHabitat();
    }
  });

  // Keyboard accessibility: Space or M triggers performance
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

    if (e.code === 'Space' || e.key === 'm' || e.key === 'M') {
      e.preventDefault();
      if (!isPerforming) {
        if (!btnMoonwalk.classList.contains('is-hidden')) {
          startPerformance(false);
        } else if (!btnReplay.classList.contains('is-hidden')) {
          startPerformance(true);
        }
      }
    }
  });

  function resetToInitial() {
    isPerforming = false;
    stopAudioSafely();
    videoStage.pause();
    videoStage.currentTime = 0;
    videoStage.classList.remove('active');
    videoStage.classList.add('hidden');
    videoIdle.classList.remove('hidden');
    videoIdle.classList.add('active');
    videoIdle.play().catch(() => {});
    btnMoonwalk.classList.remove('is-hidden');
    btnMoonwalk.classList.remove('active-walking');
    btnMoonwalk.disabled = false;
    moonwalkLabel.textContent = 'Moonwalk';
    btnReplay.classList.add('is-hidden');
    btnReplay.classList.remove('active-walking');
    btnReplay.disabled = false;
    replayLabel.textContent = 'Replay';
  }

  // Expose for external control and automated browser validation
  window.moonwalkerApp = {
    startPerformance,
    returnToHabitat,
    resetToInitial,
    playAudioSafely,
    stopAudioSafely,
    applyOrientationAssets,
    getCurrentMode: () => currentMode,
    getCurrentIdleIndex: () => currentIdleIdx,
    getNextIdleIndex,
    getIDLEVariations: () => ASSETS[currentMode].idles
  };
});
