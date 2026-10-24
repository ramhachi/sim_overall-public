# CPU portability

The default build is portable. `FSAE_ENABLE_HOST_CPU_OPTIMIZATION=OFF` does not add
`-march=native`, `-mtune=native`, or `-mcpu=native`, so binaries can run on other
machines with the same OS/ABI and a CPU that supports the compiler's baseline.

For local benchmarking only, opt in explicitly:

```bash
cmake -S . -B build-native -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFSAE_ENABLE_HOST_CPU_OPTIMIZATION=ON
cmake --build build-native
```

`FSAE_CPU_OPTIMIZATION_PROFILE` selects the native profile (`auto`, `x86-native`,
`arm-native`, `generic-native`, or `portable`). Do not distribute binaries built
with host CPU optimization enabled.

Minimum supported CPU is the compiler default for the target platform: x86-64
without additional ISA extensions on x86-64, and the platform baseline on
AArch64. Use a portable build for CI, Docker images, and release artifacts.
