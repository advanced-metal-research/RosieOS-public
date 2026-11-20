import puppeteer from "puppeteer-core";

const browser = await puppeteer.connect({
  browserURL: "http://127.0.0.1:9223",
  defaultViewport: null,
});

const pages = await browser.pages();
const page = pages.find((p) => p.url().includes("/cadquery/v1/ui/"));
if (!page) {
  console.error("no cadquery page found; open tabs: " + pages.map((p) => p.url()).join(", "));
  process.exit(1);
}

const info = await page.evaluate(() => {
  const log = document.querySelector("#report-log, .report-log, [class*='log']");
  const lines = log ? log.innerText.trim().split("\n").slice(0, 8) : [];
  return { url: window.location.href, title: document.title, logHead: lines };
});
console.log(JSON.stringify(info, null, 2));

await page.screenshot({ path: process.argv[2] ?? "chrome-loop-check.png" });
browser.disconnect();
