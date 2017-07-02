# Deformation Transfer

The deformation of the mesh in two source images (undeformed image and deformed image) is transferred to the target image, and an image after deformation of the target is generated.

![samune](https://user-images.githubusercontent.com/18544494/27771721-abcf7f14-5f8f-11e7-9525-8d937635027f.JPG)

## Enviroment
C++, Visual Studio 2015, OpenFrameworks0.9.3

### Library
Freeglut, Eigen, libigl


## Data
- Input
  - source(undeformed)　→　kawai_N.obj (top left)
  - source(deformed) → kawai_a.obj (top right)
  - target(undeformed) → mizo_N.obj (bottom left)
- Output
  - target(deformed) → mizo_a.obj (bottom right)
## References
["Deformation Transfer for Triangle Meshes",Robert W. Sumner Jovan Popovic](https://pdfs.semanticscholar.org/6a5d/cfbd6498a36976a8ea22ed21bb21601b7999.pdf)

["Mesh Modification Using Deformation Gradients", Robert Walker Sumner](http://people.csail.mit.edu/sumner/thesis/Sumner2005MMU.pdf)
