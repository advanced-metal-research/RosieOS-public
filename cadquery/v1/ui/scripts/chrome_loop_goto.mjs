import puppeteer from "puppeteer-core";

const url = process.argv[2];
const browser = await puppeteer.connect({ browserURL: "http://127.0.0.1:9223", defaultViewport: null });
const pages = await browser.pages();
const page = pages.find((p) => p.url().includes("/cadquery/v1/ui/")) ?? pages[0];
await page.goto(url, { waitUntil: "domcontentloaded" });
console.log("navigated to " + page.url());
browser.disconnect();
