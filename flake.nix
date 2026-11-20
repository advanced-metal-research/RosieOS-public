{
  description = "Reproducible RosieOS host-side build and contract checks";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      supportedSystems = [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
    in {
      packages = forAllSystems (system:
        let pkgs = import nixpkgs { inherit system; };
        in {
          motion-server-host = pkgs.stdenv.mkDerivation {
            pname = "rosie-motion-server-host";
            version = "0.1.0";
            src = ./.;
            nativeBuildInputs = [ pkgs.gnumake ];
            buildPhase = ''
              runHook preBuild
              make -C motion-server/v1 BIN_DIR="$PWD/out" OBJECT_CACHE_DIR="$PWD/cache"
              runHook postBuild
            '';
            installPhase = ''
              runHook preInstall
              install -Dm755 out/robot-v4-cartesiand "$out/bin/robot-v4-cartesiand"
              runHook postInstall
            '';
          };

          default = self.packages.${system}.motion-server-host;
        });

      checks = forAllSystems (system:
        let
          pkgs = import nixpkgs { inherit system; };
          motionServer = self.packages.${system}.motion-server-host;
        in {
          motion-server-tests = pkgs.runCommand "rosie-motion-server-tests" {
            nativeBuildInputs = [ pkgs.gnumake pkgs.stdenv.cc ];
            # Production-loop tests load the repository's pinned URDF fixtures.
            src = ./.;
          } ''
            cp -R "$src" source
            chmod -R u+w source
            make -C source/motion-server/v1 test BIN_DIR="$TMPDIR/bin" OBJECT_CACHE_DIR="$TMPDIR/cache"
            touch "$out"
          '';

          motion-server-build = motionServer;
        });

      devShells = forAllSystems (system:
        let pkgs = import nixpkgs { inherit system; };
        in {
          default = pkgs.mkShell {
            packages = [ pkgs.go pkgs.gnumake pkgs.stdenv.cc pkgs.binutils pkgs.jq ];
            shellHook = ''
              export GOTOOLCHAIN=local
              echo "RosieOS host validation shell. Deployed binaries still require the manifest-selected target toolchain and loader check."
            '';
          };
        });
    };
}
