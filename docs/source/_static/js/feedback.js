document.addEventListener('DOMContentLoaded', function () {
    const feedbackBtn = document.getElementById('feedback-btn-link');
  
    const popup = document.createElement('div');
    popup.id = 'feedback-popup';
  
    // 获取版本号（如果有的话）
    const versionElement = document.querySelector('.shibuya-footer__version');
    const currentScript = document.currentScript || document.querySelector('script[src*="feedback.js"]');
    const version = currentScript ? currentScript.getAttribute('doc-version') : '未知版本';

    // 当前页面标题
    const pageTitle = document.title;
  
    popup.innerHTML = `
      <div style="margin-bottom: 10px; font-size: 14px; color: #555;">
        页面：<strong>${pageTitle}</strong><br>
        版本：<strong>${version}</strong>
      </div>
      <textarea id="feedback-text" placeholder="请输入您的反馈..." rows="6"></textarea>
      <div style="margin-top: 20px; font-size: 13px; color: #333;">
        如果您希望我们与您取得联系，请提供以下信息：
      </div>
      <div class="flex-row" style="display: flex; gap: 20px; align-items: flex-start; margin-top: 8px;">
        <div style="display: flex; flex-direction: column; flex: 1; max-width: 200px;">
          <label for="feedback-name" style="margin-bottom: 4px;">姓名</label>
          <input type="text" id="feedback-name" placeholder="您的姓名" style="padding: 6px;" />
        </div>
        <div style="display: flex; flex-direction: column; flex: 1; max-width: 200px;">
          <label for="feedback-email" style="margin-bottom: 4px;">邮箱</label>
          <input type="email" id="feedback-email" placeholder="您的邮箱" style="padding: 6px;" />
        </div>
      </div>
      <div class="feedback-actions">
        <button id="feedback-cancel">取消</button>
        <button id="feedback-submit">提交</button>
      </div>
    `;

    document.body.appendChild(popup);
  
    feedbackBtn.onclick = (e) => {
      e.preventDefault();
      popup.style.display = 'block';
    };
  
    document.getElementById('feedback-cancel').onclick = () => popup.style.display = 'none';
    
    document.getElementById('feedback-submit').onclick = async () => {
      const submitBtn = document.getElementById('feedback-submit');
      submitBtn.disabled = true; // 禁用提交按钮
      submitBtn.textContent = '提交中...'; // 可选：显示提交中

      const name = document.getElementById('feedback-name').value.trim();
      const email = document.getElementById('feedback-email').value.trim();
      const text = document.getElementById('feedback-text').value.trim();
      if (!text) {
        alert('请填写反馈内容');
        submitBtn.disabled = false;
        submitBtn.textContent = '提交';        
        return;
      }
  
      // 如果填写了姓名或邮箱，则必须同时填写且邮箱格式正确
      if ((name && !email) || (!name && email)) {
          alert('请同时填写您的姓名和邮箱，或者两者都留空');
          submitBtn.disabled = false;
          submitBtn.textContent = '提交';          
          return;
      }

      if (email) {
          const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
          if (!emailRegex.test(email)) {
              alert('请输入有效的邮箱地址');
              submitBtn.disabled = false;
              submitBtn.textContent = '提交';              
              return;
          }
      }

      // 获取页面标题和版本号
      const pageTitle = document.title;
      const versionElement = document.querySelector('.shibuya-footer__version');
      const pageURL = window.location.href;

      // 组装请求数据
      const payload = {
          pageTitle,
          pageURL,
          version,
          feedback: text,
          name,
          email,
      };

      try {
        const response = await fetch('https://feedback.sifli.com/send_feedback', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(payload)
        });

        const result = await response.json();

        if (response.ok && result.success) {
            alert('感谢您的反馈！我们会尽快处理。');
            // 清空输入框并关闭弹窗
            document.getElementById('feedback-text').value = '';
            document.getElementById('feedback-name').value = '';
            document.getElementById('feedback-email').value = '';
            popup.style.display = 'none';
        } else {
            alert(result.message || '提交失败，请稍后重试。');
        }
      } catch (error) {
          alert('网络异常，提交失败，请稍后再试。');
      } finally {
        submitBtn.disabled = false;
        submitBtn.textContent = '提交';
      }
    };
  });
  