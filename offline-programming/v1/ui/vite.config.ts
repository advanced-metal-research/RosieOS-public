import { defineConfig } from "vite";

const base = "/offline-programming/v1/ui/";

export default defineConfig({
  base,
  publicDir: ".staged-public",
  server: {
    host: "127.0.0.1",
    port: 5189,
    allowedHosts: true,
    proxy: {
      "/api/offline-programming/v1": {
        target: process.env.OFFLINE_PROGRAMMING_API_URL ?? "http://127.0.0.1:8794",
        changeOrigin: false,
      },
    },
  },
  build: {
    outDir: "dist",
    emptyOutDir: true,
    chunkSizeWarningLimit: 900,
    rollupOptions: {
      output: {
        manualChunks(id: string) {
          if (id.includes("node_modules/three/") || id.includes("node_modules/urdf-loader/")) {
            return "three";
          }
          return undefined;
        },
      },
    },
  },
});

