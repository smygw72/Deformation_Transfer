# Deformation Transfer

The deformation of the mesh in two source images (undeformed image and deformed image) is transferred to the target image, and an image after deformation of the target is generated.

## Enviroment
C++, Visual Studio 2015, OpenFrameworks0.9.3

### Library
Freeglut, Eigen, libigl


## Data
- Input
  - source(undeformed)　→　kawai_N.obj
  - source(deformed) → kawai_a.obj
  - target(undeformed) → mizo_N.obj
- Output
  - target(deformed) → mizo_a.obj

## Reference
["Deformation Transfer for Triangle Meshes",Robert W. Sumner Jovan Popovic](https://pdfs.semanticscholar.org/6a5d/cfbd6498a36976a8ea22ed21bb21601b7999.pdf)

["Mesh Modification Using Deformation Gradients", Robert Walker Sumner](http://people.csail.mit.edu/sumner/thesis/Sumner2005MMU.pdf)
