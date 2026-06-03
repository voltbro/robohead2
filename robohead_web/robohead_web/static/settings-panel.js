class SettingsPanel {
constructor() {
  // ===== Элементы токена =====
  this.yaTokenInput = document.getElementById('yaTokenInput');
  this.editBtn = document.getElementById('editBtn');
  this.saveBtn = document.getElementById('saveBtn');
  this.cancelBtn = document.getElementById('cancelBtn');
  this.authBtn = document.getElementById('authBtn');
  this.yaAuthStatus = document.getElementById('yaAuthStatus');
  
  // Элементы модального окна
  this.authModal = document.getElementById('authModal');
  this.modalCloseBtn = document.getElementById('modalCloseBtn');
  this.authCodeDisplay = document.getElementById('authCodeDisplay');
  this.copyCodeBtn = document.getElementById('copyCodeBtn');
  this.authUrlLink = document.getElementById('authUrlLink');
  this.authPollStatus = document.getElementById('authPollStatus');

  // ===== Элементы громкости =====
  this.volumeInput = document.getElementById('volumeInput');
  this.editVolBtn = document.getElementById('editVolBtn');
  this.saveVolBtn = document.getElementById('saveVolBtn');
  this.cancelVolBtn = document.getElementById('cancelVolBtn');
  this.volStatus = document.getElementById('volStatus');

  // ===== Состояние =====
  this.originalToken = '';
  this.originalVolume = 60;
  this.currentPollId = null;
  this.pollInterval = null;

  // ===== Инициализация (вызываем ОДИН РАЗ) =====
  this.setupEventListeners();
  this.loadToken();
  this.loadVolume();
}

  setupEventListeners() {
    this.editBtn.addEventListener('click', () => this.startEditing());
    this.saveBtn.addEventListener('click', () => this.saveToken());
    this.cancelBtn.addEventListener('click', () => this.cancelEditing());
    this.authBtn.addEventListener('click', () => this.startDeviceAuth());
    this.modalCloseBtn.addEventListener('click', () => this.closeModal());
    
    // Закрытие по клику на оверлей
    this.authModal.addEventListener('click', (e) => {
      if (e.target === this.authModal) this.closeModal();
    });

    this.copyCodeBtn.addEventListener('click', () => this.copyCode());

    this.editVolBtn.addEventListener('click', () => this.startEditingVolume());
    this.saveVolBtn.addEventListener('click', () => this.saveVolume());
    this.cancelVolBtn.addEventListener('click', () => this.cancelEditingVolume());
  }

  loadVolume() {
    this.setVolStatus('Загрузка...', '#888');
    fetch('/api/settings/default-volume')
      .then(r => r.json())
      .then(data => {
        this.originalVolume = data.volume;
        this.volumeInput.value = this.originalVolume;
        this.setVolStatus('✓ Значение загружено из конфига', '#6ee7b7');
      })
      .catch(e => this.setVolStatus('✗ Ошибка загрузки', '#ff6b6b'));
  }

 /** Включает режим редактирования */
  startEditingVolume() {
    this.volumeInput.removeAttribute('readonly');
    this.volumeInput.focus();
    this.editVolBtn.classList.add('hidden');
    this.saveVolBtn.classList.remove('hidden');
    this.cancelVolBtn.classList.remove('hidden');
    this.setVolStatus('Введите значение (0-100)', '#888');
  }

  /** Отменяет изменения */
  cancelEditingVolume() {
    this.volumeInput.value = this.originalVolume;
    this.volumeInput.setAttribute('readonly', true);
    this.editVolBtn.classList.remove('hidden');
    this.saveVolBtn.classList.add('hidden');
    this.cancelVolBtn.classList.add('hidden');
    this.setVolStatus('Изменения отменены', '#888');
  }

  /** Сохраняет громкость на сервер */
  saveVolume() {
    const newVol = parseFloat(this.volumeInput.value);
    if (isNaN(newVol) || newVol < 0 || newVol > 100) {
      return this.setVolStatus('❌ Допустимый диапазон: 0-100', '#ff6b6b');
    }

    this.setVolStatus('Сохранение...', '#888');
    this.saveVolBtn.disabled = true;

    fetch('/api/settings/default-volume', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ volume: newVol })
    })
      .then(r => r.json())
      .then(data => {
        if (data.status === 'success') {
          this.originalVolume = newVol;
          this.cancelEditingVolume(); // Выходим из режима редактирования
          this.setVolStatus('✅ Громкость обновлена в robohead_controller/config/media_driver.yaml', '#6ee7b7');
        } else {
          this.setVolStatus('✗ ' + (data.message || 'Ошибка'), '#ff6b6b');
        }
      })
      .catch(() => this.setVolStatus('✗ Ошибка сети', '#ff6b6b'))
      .finally(() => this.saveVolBtn.disabled = false);
  }

  setVolStatus(text, color) {
    this.volStatus.textContent = text;
    this.volStatus.style.color = color;
  }

  /** Загрузка токена при старте */
  loadToken() {
    this.setStatus('Проверка настроек...', '#888');
    fetch('/api/settings/ya-token')
      .then(r => r.json())
      .then(data => {
        this.originalToken = data.token || '';
        this.yaTokenInput.value = this.originalToken;
        this.setStatus(this.originalToken ? '✓ Токен загружен из .env' : 'Токен не задан', this.originalToken ? '#6ee7b7' : '#ffa500');
      })
      .catch(e => this.setStatus('✗ Ошибка связи с сервером', '#ff6b6b'));
  }

  /** Включает режим редактирования */
  startEditing() {
    this.yaTokenInput.removeAttribute('readonly');
    this.yaTokenInput.focus();
    this.editBtn.classList.add('hidden');
    this.saveBtn.classList.remove('hidden');
    this.cancelBtn.classList.remove('hidden');
    this.setStatus('Режим редактирования', '#888');
  }

  /** Отменяет редактирование */
  cancelEditing() {
    this.yaTokenInput.value = this.originalToken;
    this.yaTokenInput.setAttribute('readonly', true);
    this.editBtn.classList.remove('hidden');
    this.saveBtn.classList.add('hidden');
    this.cancelBtn.classList.add('hidden');
    this.setStatus('Изменения отменены', '#888');
  }

  /** Сохраняет токен вручную */
  saveToken() {
    const newToken = this.yaTokenInput.value.trim();
    if (!newToken) return this.setStatus('Токен не может быть пустым', '#ff6b6b');

    this.setStatus('Сохранение...', '#888');
    this.saveBtn.disabled = true;

    fetch('/api/settings/ya-token', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ token: newToken })
    })
      .then(r => r.json())
      .then(data => {
        if (data.status === 'success') {
          this.originalToken = newToken;
          this.cancelEditing(); // Выходим из режима редактирования
          this.setStatus('✓ Токен сохранен в .env', '#6ee7b7');
        } else {
          this.setStatus('✗ ' + (data.message || 'Ошибка'), '#ff6b6b');
        }
      })
      .catch(() => this.setStatus('✗ Ошибка сети', '#ff6b6b'))
      .finally(() => this.saveBtn.disabled = false);
  }

  /** Запуск авторизации через код */
  startDeviceAuth() {
    if (!this.yaTokenInput.hasAttribute('readonly')) this.cancelEditing();
    
    this.authBtn.disabled = true;
    this.authBtn.innerHTML = '⏳';
    this.setStatus('Инициализация...', '#888');

    fetch('/api/settings/ya-auth-init', { method: 'POST' })
      .then(r => r.json())
      .then(data => {
        if (data.poll_id) {
          this.showAuthModal(data.user_code, data.verification_url, data.poll_id);
        } else {
          this.onAuthError(data.error || 'Не удалось получить код');
        }
      })
      .catch(() => this.onAuthError('Ошибка сети'))
      .finally(() => { this.authBtn.disabled = false; this.authBtn.innerHTML = '🔑'; });
  }

  showAuthModal(code, url, pollId) {
    this.currentPollId = pollId;
    this.authCodeDisplay.textContent = code;
    this.authUrlLink.href = url;
    this.authPollStatus.className = 'poll-status';
    this.authPollStatus.innerHTML = '<span class="spinner"></span> Ожидание ввода кода...';
    this.copyCodeBtn.classList.remove('copied');
    this.copyCodeBtn.textContent = 'Копировать';
    
    this.authModal.classList.remove('hidden');
    this.startPolling(pollId);
  }

  startPolling(pollId) {
    let attempts = 0;
    const maxAttempts = 180; // 3 минуты
    if (this.pollInterval) clearInterval(this.pollInterval);

    this.pollInterval = setInterval(() => {
      if (attempts >= maxAttempts) {
        this.stopPolling();
        this.onAuthError('Время ожидания истекло');
        return;
      }

      fetch('/api/settings/ya-auth-poll', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ poll_id: pollId })
      })
        .then(r => r.json())
        .then(data => {
          if (data.ready) { this.stopPolling(); this.onAuthSuccess(data.token); }
          else if (data.error) { this.stopPolling(); this.onAuthError(data.error); }
        })
        .catch(() => {}); // Игнорируем редкие сетевые сбои во время поллинга

      attempts++;
    }, 1500);
  }

  stopPolling() {
    if (this.pollInterval) clearInterval(this.pollInterval);
    this.pollInterval = null;
  }

  onAuthSuccess(token) {
    this.authPollStatus.className = 'poll-status success';
    this.authPollStatus.textContent = '✓ Токен успешно получен!';
    
    // Автозаполнение и обновление состояния
    this.originalToken = token;
    this.yaTokenInput.value = token;
    this.yaTokenInput.setAttribute('readonly', true);
    this.editBtn.classList.remove('hidden');
    this.saveBtn.classList.add('hidden');
    this.cancelBtn.classList.add('hidden');
    
    this.setStatus('✓ Токен получен и сохранен', '#6ee7b7');
    setTimeout(() => this.closeModal(), 1500);
  }

  onAuthError(message) {
    this.authPollStatus.className = 'poll-status error';
    this.authPollStatus.textContent = `✗ ${message}`;
    this.setStatus(`✗ ${message}`, '#ff6b6b');
  }

  closeModal() {
    this.authModal.classList.add('hidden');
    this.stopPolling();
    this.currentPollId = null;
  }

  copyCode() {
    navigator.clipboard.writeText(this.authCodeDisplay.textContent).then(() => {
      this.copyCodeBtn.classList.add('copied');
      this.copyCodeBtn.textContent = 'Скопировано!';
      setTimeout(() => {
        this.copyCodeBtn.classList.remove('copied');
        this.copyCodeBtn.textContent = 'Копировать';
      }, 2000);
    });
  }

  setStatus(text, color) {
    this.yaAuthStatus.textContent = text;
    this.yaAuthStatus.style.color = color;
  }
}

document.addEventListener('DOMContentLoaded', () => { window.settingsPanel = new SettingsPanel(); });