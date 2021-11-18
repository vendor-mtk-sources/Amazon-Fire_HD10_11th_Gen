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
You will need the files platform.tar and build_kernel.tar.gz from the
tarball to build the kernel.

1.  Extract the build_kernel.tar.gz tarball to a directory of your choosing.

2.  Obtain a copy of Clang compiler. Recommended version: 6.0.2.

3.  Update build_kernel_config.sh and paste in the path to the root of the
    copy of the Clang compiler in the CLANG_COMPILER_PATH variable.

4.  Execute the script by running:

    build_kernel.sh "<path to platform.tar>" "<target output directory>"


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
the libraries, we used the target "aosp_walleye-userdebug" as the target of the
lunch command.

5. Upon completion of the build, the relevant libraries can be found under the
'out/target' folder of the source code.
