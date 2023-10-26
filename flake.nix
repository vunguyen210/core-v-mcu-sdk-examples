{
  inputs = {
    nixpkgs.url = "nixpkgs/nixpkgs-unstable";
  };
  outputs = { self, nixpkgs }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      eachSystem = f: with nixpkgs.lib; foldAttrs mergeAttrs { }
        (map (s: mapAttrs (_: v: { ${s} = v; }) (f s)) systems);
    in
    eachSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        inherit (pkgs) lib fd;
        format_tools = with pkgs; [
          clang-tools_16
          coreutils
          cmake-format
          nixpkgs-fmt
          nodePackages.prettier
        ];
        format_cmd = pkgs.writeShellScript "format" ''
          PATH=${lib.makeBinPath format_tools}
          case "$1" in
            *.c | *.h)
              clang-format -i "$1";;
            *.nix)
              nixpkgs-fmt "$1";;
            *.md | *.json | *.yml)
              prettier --write "$1";;
          esac &>/dev/null
        '';
      in
      rec {
        formatter = pkgs.writeShellScriptBin "formatter" ''
          for f in "$@"; do
            if [ -d "$f" ]; then
              (cd "$f"; ${fd}/bin/fd -t f -H -E '**/lib/*/*' -E '**/common-io-basic/*' -E '**/FreeRTOS-Kernel/*' -E '**/tests/*' -E '**/shared/*' -x ${format_cmd})
            else
              ${format_cmd} "$f"
            fi
          done
        '';
      });
}
