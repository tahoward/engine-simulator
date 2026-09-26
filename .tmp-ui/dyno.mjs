import { chromium } from 'playwright-core';
const browser = await chromium.launch({ args: ['--autoplay-policy=no-user-gesture-required'] });
const page = await browser.newPage({ viewport: { width: 1400, height: 900 } });
page.on('pageerror', (e) => console.log('PAGEERROR', e.message));
page.on('console', (m) => { if (m.type() === 'error') console.log('CONSOLE', m.text()); });
await page.goto('http://localhost:5199/');
await page.waitForTimeout(1000);
await page.click('#overlay');
await page.waitForTimeout(1500);
await page.evaluate(() => {
  const sel = [...document.querySelectorAll('select')].find((s) => [...s.options].some((o) => o.textContent === 'Inline four'));
  sel.value = [...sel.options].find((o) => o.textContent === 'Inline four').value;
  sel.dispatchEvent(new Event('change'));
});
await page.waitForTimeout(2500);
await page.evaluate(() => {
  const r = [...document.querySelectorAll('.slider-row')].find((x) => x.querySelector('label')?.textContent === 'Car mass');
  const i = r.querySelector('input'); i.value = '300'; i.dispatchEvent(new Event('input'));
});
await page.evaluate(() => [...document.querySelectorAll('button')].find((b) => b.textContent === 'Start dyno run').click());
await page.waitForTimeout(6000);
const card = page.locator('.dyno-card');
await card.screenshot({ path: '.tmp-ui/mid.png' });
console.log('status mid:', await page.textContent('.dyno-status'));
for (let i = 0; i < 40; i++) {
  await page.waitForTimeout(1000);
  const st = await page.textContent('.dyno-status');
  if (st.startsWith('Run')) { console.log('status end:', st, 'after', i + 7, 's'); break; }
}
console.log('peaks:', await page.textContent('.dyno-peaks'));
const box = await page.locator('.dyno-plot canvas').boundingBox();
await page.mouse.move(box.x + box.width * 0.7, box.y + box.height * 0.4);
await page.waitForTimeout(300);
await card.screenshot({ path: '.tmp-ui/end.png' });
console.log('button:', await page.evaluate(() => [...document.querySelectorAll('button')].find((b) => b.textContent.includes('dyno run')).textContent));
await browser.close();
