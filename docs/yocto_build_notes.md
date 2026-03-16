# Yocto Build Notes

## 1. Goal
This documents defines the build cache stategy and reproducibility rules for Yocto-based environment setup.

## 2. What was validated this week
- cache strategy (DL_DIR / SSTATE_DIR)
- build log retention
- first successful baseline build
- minimal runtime package extension
- rebuild with cache reuse
- run_info template standardization

## 3. Cache directories
- DL_DIR : downloads source cache
- SSTATE_DIR : shared stae build cache

## 4. Cache directory paths
- DL_DIR : ~/yocto_cache/downloads
- SSTATE_DIR : ~/yocto_cache/sstate_cache

## 5. Result artifacts
- build logs
- manifest
- run_info.txt
- package baseline comparison

## 6. Reproducible build steps

### Step 1. Prepare cache directories
- create `$HOME/yocto_cache/downloads`
- create `$HOME/yocto_cache/sstate-cache`

### Step 2. Clone Yocto poky
- clone `poky`
- checkout `kirkstone`

### Step 3. Initialize build environment
- run `source oe-init-build-env build`

### Step 4. Configure cache paths
- set `DL_DIR`
- set `SSTATE_DIR`

### Step 5. Run the first baseline build
- build `core-image-minimal`
- save full build log

### Step 6. Extend the minimal runtime package set
- add `python3 openssh iproute2 procps`
- rebuild
- verify package presence through manifest

### Step 7. Save evidence
- copy manifest into `results/`
- keep build log
- update `run_info.txt`