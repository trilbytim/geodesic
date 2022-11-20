{
  name,
  availableKernels,
  extraArgs,
}:
availableKernels.python {
  name = "withbatteries";
  inherit (extraArgs) pkgs;
  displayName = "Custom ${name}";
  extraPackages = ps: [ ps.pillow ];
}
