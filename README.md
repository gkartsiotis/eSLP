# eSLP

A. Description

The m-files in this repository implement the eSLP algorithm. The documentation that follows addresses the "core" issues; requirements, and how to regarding the setup and running the provided example (a moving obstacle on a terrain that changes through time). Now, we are working on modifying the m-files because they are tailor-made for specific experiments, e.g., dynamics, cost functions, and require that they along with compiler related files are installed in predetermined locations in the local drive. These modifications will be applied the current files along to those that model two other environments we have already examined which include moving deforming obstacles, and a bridge crane system. 

B. Requirements

To run the m-files you will need:
1. Matlab (We used version 2019b)
2. Falcon.m found here https://www.fsd.lrg.tum.de/software/falcon-m/ (We used v1.24.2002191427)
3. TDM-GCC compiler (We used TDM-GCC Compiler Suite for Windows GCC 5 Series / GDB 7.9 Series and installed the MinGW-w64 64/32-bit Edition)
4. There are two additional files needed for the compiler to run smoothly (This process may have changed for versions after 2019b); both are included in the repository. The first one is configuremingw.p information for which can be found here https://www.mathworks.com/matlabcentral/answers/313298-i-already-have-mingw-on-my-computer-how-do-i-configure-it-to-work-with-matlab . The second is compile.m needed to set the environment variable properly.

C.1. How-To : Setup
1. Request for a copy of Falcon.m from B2. The support is great and one will be send soon. In the zip you will receive there are well-writen instructions in file UserGuideMain.pdf (Chapter 2).
2. Open matlab and run the setup required by Falcon.m (the process is thoroughly explained in the accompanying manual). Falcon.m will automatically download many of the packages needed, e.g., IPOPT solver. We installed Falcon on folder C:\FALCON.m.v1.24.2002191427\falcon and the m-files in this repository use this to perform clean up which while not required it offers backwards compatibility and stability (This issue may not exist in newer versions of the compiler and/or Matlab in which the corresponding lines at the end of the m-files can be removed).

C.2. How-To : Run example
1. Browse to the folder where Falcon was installed, create a subfolder, and place the files from this repository there. In general we suggest that the files for every experiment are placed in a different subfolder of the main Falcon folder (the one where you installed Falcon). For example, the files on this repository should be placed on folder C:\FALCON.m.v1.24.2002191427\falcon\SetupA.
2. Run file configuremingw.p and select the folder where the compiler is. We installed GCC on C:\TDM-GCC-64 and suggest this is the one used.
3. Run file compile.m. This sets the "environment" variable. We installed the necessary compiler files in C:\mingw-w64\x86_64-5.3.0-posix-seh-rt_v4-rev0\mingw64. If a different location is used change accordingly in the compile.m (first line - setenv command)
4. Run main.m and afterwards FinalResults.m.
