[![build and test](https://github.com/technoroad/TR_IMU_LIB/actions/workflows/ci_test.yml/badge.svg)](https://github.com/technoroad/TR_IMU_LIB/actions/workflows/ci_test.yml)

# Build and Test
```sh
mkdir  build
cd build
cmake -DRUN_TEST=1 ..
make
ctest --verbose
```
