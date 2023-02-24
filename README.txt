README
======
THESE INSTRUCTIONS AND RELATED BUILD SCRIPTS ARE PROVIDED BY AMAZON ON AN
"AS IS" BASIS. AMAZON MAKES NO REPRESENTATIONS OR WARRANTIES OF ANY KIND,
EXPRESS OR IMPLIED, AS TO THESE INSTRUCTIONS, RELATED BUILD SCRIPTS, OR ANY
THIRD PARTY TECHNOLOGY SUCH AS ANDROID OPEN SOURCE PROJECT CODE OR THIRD PARTY
COMPILERS REFERENCED THEREIN (COLLECTIVELY, “BUILD MATERIALS”). YOU EXPRESSLY
AGREE THAT YOUR USE OF THE BUILD MATERIALS IS AT YOUR SOLE RISK.

AMAZON WILL NOT BE LIABLE FOR ANY DAMAGES OF ANY KIND ARISING FROM THE USE OF
THE BUILD MATERIALS INCLUDING, BUT NOT LIMITED TO, DIRECT, INDIRECT,
INCIDENTAL, PUNITIVE, AND CONSEQUENTIAL DAMAGES.


BUILDING THE KERNEL
-------------------

1. You may need to install prerequisite libraries. On a Debian-based system:

    $ sudo apt-get install -y \
        git gnupg flex bison gperf build-essential zip curl \
        zlib1g-dev gcc-multilib g++-multilib libc6-dev-i386 lib32ncurses5-dev \
        x11proto-core-dev libx11-dev lib32z-dev ccache libgl1-mesa-dev \
        libxml2-utils xsltproc unzip python lib32z1 lib32stdc++6 libssl-dev \
        libswitch-perl swig maven libncurses5 xxd bc vim

2. You may need to install pycryptodome

   $ curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py
   $ python2 get-pip.py
   $ pip install pycryptodome

3. You may need to change /bin/sh to bash:

    sudo mv /bin/sh /bin/sh.orig
    sudo ln -s /bin/bash /bin/sh

4. Build may use prebuilt binary minigzip, a 32-bit binary, and if you are
   using 64-bit Linux, you may need to install additional libraries:

        Ubuntu 12.04:
        sudo sh -c "echo 'foreign-architecture i386' > /etc/dpkg/dpkg.cfg.d/multiarch"
        sudo apt-get update
        sudo apt-get install multiarch-support
        sudo apt-get install libc6:i386 libncurses5:i386 libstdc++6:i386

5.  Check build_kernel_config.sh if any additional compilers are needed.

6.  Execute the script by running:

    ./build_kernel.sh "<path to platform.tar>" "<target output directory>"


BUILDING BUSYBOX (if applicable)
--------------------------------

1.  Check build_busybox_config.sh if any additional compilers are needed.

2.  Execute the script by running:

    ./build_busybox.sh "<path to platform.tar>" "<target output directory>"


BUILDING UBOOT (if applicable)
------------------------------

1.  Check build_uboot_config.sh if any additional compilers are needed.

2.  Execute the script by running:

    ./build_uboot.sh "<path to platform.tar>" "<target output directory>"


BUILDING LIBRARIES
------------------
The libraries are intended to be built within the context of the Android Open
Source Project's source code.

1. Set up a build environment per the instructions in
https://source.android.com/source/initializing.html  Set JAVA_HOME in your
execution environment to the root of the installed JDK.

2. Download repo and the source code for AOSP per the instructions in
https://source.android.com/source/downloading.html .  For the libraries
packaged in this tarball, you should use "android-9.0.0_r1" as the tagged branch
for checkout, by passing the value above into repo with the -b switch.

3. After completing the 'repo sync' command in the above instructions, copy in
the source code for the libraries that you wish to build in place over the
checked out source code, modifying files in place if needed.

4. Build the source code per the instructions in
https://source.android.com/source/building.html .  For the purposes of testing
the libraries, we used the target "aosp_walleye-userdebug" as the target of the 'lunch'
command.

5. Upon completion of the build, the relevant libraries can be found under the
'out/target' folder of the source code.
