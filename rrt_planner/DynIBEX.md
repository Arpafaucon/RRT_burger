# DynIBEX

Report of installation
Instructions : http://perso.ensta-paristech.fr/~chapoutot/dynibex/index.php#download-installation

## IBEX

Downloaded 2.1.7 version mirrored .
Requirements : 
```sh
sudo apt-get install -y python2.7 flex bison gcc g++ make pkg-config
```

In folder `DynIbex/ibex-2.1.17` :
```sh
./waf configure
sudo ./waf install
```

Testing `foo.cpp` compilation. 
In examples/, create foo.cpp
```cpp
#include "ibex.h"
#include <iostream>

using namespace std;
using namespace ibex;

int main(int argc, char** argv) {
  Interval x(0,1);
  cout << "My first interval: "<< x << endl;
}
```

Then 
```sh
make foo
./foo
```

Expected result : 
```
My first interval: [0, 1]
```

## DynIbex

decompress the source code into the same `DynIbex/ibex-2.1.17` folder. You should have :
```sh
ls
3rd      benchs     CHANGES         example_dae.cpp      example_integrate.cpp  flex.py   install_plugin.py  LICENSE       README  tmp_install         VERSION  wscript
AUTHORS  __build__  COPYING.LESSER  example_dae_ctc.cpp  examples               flex.pyc  integrate          pendulum.cpp  src     tmp_install.tar.gz  waf
```

Then, repeat installation 
```sh
./waf configure
sudo ./waf install
```

