# Detection and Refinement of Orthogonal Plane Pairs and Derived Orthogonality Primitives

This repository provides the code accompanying the paper
_From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds_
by C. Sommer, Y. Sun, L. Guibas, D. Cremers and T. Birdal,
published in IEEE Robotics and Automation Letters (RA-L) 2020 and presented at the International Conference on Robotics and Automation (ICRA) 2020.
The paper is available at [IEEE Xplore](https://doi.org/10.1109/LRA.2020.2969936), and a preprint can be found on [arXiv](https://arxiv.org/abs/2001.07360).
The [RA-L video](https://youtu.be/nHWJrA6RcB0) (2:30) and the [ICRA presentation video](https://youtu.be/hcdCKUh1d8U) (10:00) are on YouTube.
The code in this repository is a basic demonstration of how our method works and one possible option to implement it.
We simplified some things compared to our original implementation (e.g. subsampling and clustering), in order to not depend on code that we cannot publish.
The core functionality still remains the same.


### Method Overview

##### Plane Pair Detection

The core idea of our method is to employ a semi-global Hough voting scheme in order to extract orthogonal plane pairs:
for a plane in 3D defined by a reference point + normal, the space of possible orthogonal planes reduces to two dimensions, so we can pair the reference point with a set of other points and create a 2D voting array to see if there are any orthogonal planes.

```c++
init graph of planes and orthogonality relations
for all reference points xr
    init voting table
    choose set of pair points
    for all pair points xi
        compute F(xr,xi)
        if |F1| < threshold
            compute (rho, theta)
            vote
        end
    end
    extract max in voting table
    add plane pair to graph
end
```

##### Refinement

We propose two refinement modes:
1. Extraction of corners and subsequent refinement of the individual corners, which can then be used for further processing (coarse scan alignment, dimensionality-reduced ICP, etc.)
2. Graph reduction and refinement of the whole graph configuration, to obtain an aligned abstraction of the scene.

### Basic Usage

##### Dependencies

The code depends on the following third-party libraries:

- Eigen (header-only)
- Ceres
- Pangolin
- Sophus (header-only)
- CLI (header-only)
- nanoflann (header-only)
- tinyply

All of these libraries are added to this repository as submodules, or directly as source files (nanoflann and tinyply).

##### Preparation

- Clone the repository to your computer including all submodules.
- Build Ceres and Pangolin in folders `3rdParty/build-ceres-solver/` and `3rdParty/build-Pangolin/`.
- Compile the code using the `CMakeLists.txt` file:
    ```bash
    mkdir build
    cd build
    cmake ..
    make -j4
    cd ..
    ```
- The current version of the code only works with single precision `ply` input. We might add further input options at a later point, but for now make sure to have the right input data format. We provide an example point cloud in `data/test_single.ply`.

##### Parameters

There is only one required input parameter - the filename of the input data file, which must be placed in the `data/` folder.
It is specified by the `--img [filename]` option.
For all optional input parameters, call `--help` to see a description.
The standard settings assume a point cloud that needs downsampling by a factor of approx. 50, so VGA or QVGA resolution input point clouds are a good choice.

##### Corner Detection and Refinement

Example usage:

```bash
    cd ply_corners # go to corner detection/refinement directory
    bin/PLY_Corners --img test_single.ply # run code
```

##### Planar-Orthogonal Scene Abstraction

Example usage:

```bash
    cd ply_detect_refine # go to scene abstraction directory
    bin/PLY_PPDetectRefine --img test_single.ply # run code
```

### License and Publication

Our code is released under the BSD-3 license, for more details please see the `LICENSE` file.
Also note the different licenses of the submodules in the folder `3rdParty` and the license of the test data, which is a point cloud adapted from the [ICL-NUIM dataset](http://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html).

Please cite our paper when using the code in a scientific project.
You can copy-paste the following BibTex entry:
```
@article{sommer2020,
    title   = {From Planes to Corners: Multi-Purpose Primitive Detection in Unorganized 3D Point Clouds},
    author  = {Sommer, Christiane and Sun, Yumin and Guibas, Leonidas and Cremers, Daniel and Birdal, Tolga},
    journal = {IEEE Robotics and Automation Letters (RA-L)},
    volume  = {5},
    number  = {2},
    pages   = {1764--1771},
    doi     = {10.1109/LRA.2020.2969936},
    year    = {2020}
}
```
