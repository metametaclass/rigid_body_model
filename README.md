# Physical model of rigid body motion with gsl ODE solver


* [Physically Based Modeling](https://www.cs.cmu.edu/~baraff/pbm/pbm.html)
* [Rigid Body Dynamics](https://www.cs.cmu.edu/~baraff/pbm/rigid1.pdf)
* [gsl](https://www.gnu.org/software/gsl/doc/html/ode-initval.html)

## Build with msys2/mingw64:

```
pacman -S --needed base-devel mingw-w64-x86_64-toolchain
pacman -S mingw-w64-x86_64-gsl
git clone ...
cd rigid_body_model
make run
```


---

## Build and debug with gcc on ubuntu linux

```
# install gsl packages
sudo apt-get install libgsl-dev

# clone project
git clone ...
cd rigid_body_model

# build and run
make run

