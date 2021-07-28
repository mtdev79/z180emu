## z180emu
A portable full system emulator of Z180 based boards

## Motivation
The goal of this project is to provide a fast, complete and accurate emulated development environment for Z180 boards. It is especially useful for BIOS, OS and ROM debugging, but can also be used to run and develop user programs.

## Build status
**build passing**
There is no CI yet.  
Supported targets are MinGW and Linux. gcc-win64-mingw64 and gcc-linux-amd64 are actively developed and tested. gcc-win32-mingw32, gcc-linux-i686 and gcc-linux-armhf should work without any issues, but are not tested.  
Currently used compiler versions:  
x86_64-w64-mingw32 gcc version 6.2.0 (Rev2, Built by MSYS2 project)	- used for primary development  
x86_64-linux-gnu gcc version 6.3.0 20170516 (Debian 6.3.0-18+deb9u1)  


## Code style
C89  
I convert all upstream C++ code to pure C. I generally find emulators easier to write and debug in C than C++.  
C is also more portable as there is no libstdc++ dependency.  

## Screenshots

## Tech/framework used
gcc  
GNU make  

## Features
### General  
**Fairly well-debugged and accurate Z180/Z182 core**  
-2 PRT timers  
-2channel DMA  
-2channel ASCI **(new!)**  
-Z182 mode enables additional peripherals **(new!)**: PA,PB,PC, ROMBR/RAMUBR/RAMLBR, 80230 2channel ESCC. (16550 MIMIC is unimplemented but can be added. IOCS is hardcoded)  
**FDC37C665 SuperIO**  
-4port NEC uPD-765 FDC (with both PIO and DMA support)  
-3.5/5.25 FDD with many DD and HD formats  
-fdimage format: raw IMG/IMA/BIN/VFD (additional such as IMD, FDI, D86F can be ported)  
-dual 16550  
-IEEE1284 LPT (unimplemented)  
-IDE ATA controller (purposefully disabled, 16bit only)  
**standalone 16-bit ATA, 8-bit XT and 8-bit latched PIO IDE**  
-CompactFlash, GIDE, ZIDE (other can be ported)  
**DS1202/1302 RTC with NVRAM file storage**  
**Z180 dissasembler and simple yet powerful tracer**  
Serial ports are implemented as **byte-oriented streams over a raw TCP socket** (works with Putty, nc, ckermit etc.)  
**Modular structure** allowing adding new boards and peripherals with reasonable effort  


### Boards/ROM/OS support  
**P112 (stable):**  
-original Dave Brooks' 19960513 and 19970308 ROM - not working, TODO  
-Hal Bower's 19970712 4.1 ROM - OK  
-Hector Peraza's 20060205 and 20070217 5.1 ROM - OK  
-Terry Gulczynski/Wayne Warthen's 20161213 5.8 ROM - OK  
-original Dave Brooks' 19970308 CP/M 2.2 image - OK  
-Hector Peraza's CP/M 2.2 20070805 image - only works with 4.x ROM, probably CP/M BIOS bug  
-Hector Peraza's CP/M 3.0 / ZPM3 20070817 image	- OK  
-Hector Peraza's MP/M 2.1 20070817 image - OK  
-Hector Peraza's UZI180 20070805 image - OK  
-Hector Peraza's RSX180 20181105 tt image - OK  
-David Griffith's 20111130 ZSDOS image - OK  
-Terry Gulczynski's 20170130 ZSDOS image - OK  
**MarkIV (alpha):**  
-UNABIOS 3.47beta with UNACPM (512k ROM) - OK  


### TODO  
Emulator console ala SimH  
Change fdimage at runtime  
Interactive debugger (display/edit/disasm/asm/single/breakpoint)  
SDcard emulation  
Clock-accurate emulation of FDC 16550 and ASCI  
LPT?  

## Installation
```
git clone
make
```

You get the p112, markiv and makedisk binaries.

## API Reference

## Tests

## How to use?
Run without trace:  
P112:  
copy your rom to p112rom.bin  
copy your fdd boot image to p112-fdd1.img  
[optional] create a hdd image using makedisk and cpmtools, and name it ide00.dsk  
```
p112
```


The emulator opens with:
```
z180emu v1.0 P112
Serial port 0 listening on 10180
Serial port 1 listening on 10181
```
waits until both sockets are connected, and then starts execution.

---  
Mark IV:  
copy your rom to markivrom.bin  
[optional] create a CF image using makedisk and cpmtools, and name it cf00.dsk  
```
markiv
```
and connect to the console socket.  
Non-LBA drives do not report capacity correctly. This seems to be a DualIDE driver issue.  


---
Run with trace:  
```
p112 d >mytrace.log
```
Then grep for it.


Run with trace from 10000000-th instruction:  
```
p112 d 10000000 >mytrace.log
```
The above can be used to bisect into the routine you're debugging.  

---
Exiting the emulator  
CTRL+C/SIGINT is completely disabled to allow ^C passthrough to the emulated system, esp. in case socket console isn't used.  
The emulator can be exited by CTRL+Break (or closing the window) on MinGW, and CTRL+\ (SIGQUIT) on POSIX systems.  

---
Socket console  
If using Putty, set Connection type to "Raw".  
For nc, use  
```
stty raw -echo; nc -C 127.0.0.1 10180
```

## Contribute
If you'd like to commit, please contact me (Michal Tomek) at <mtdev79b@gmail.com>

## Credits
Hector Peraza, for his ROM archive, MP/M, UZI and RSX180 OS images, all of which proved to be great tools for testing the emulator.  
Developers of open source code included in this project, most notably:  
-Portable Z180 emulator and Portable Z8x180 disassembler Copyright by Juergen Buchmueller  
-Z80-SCC Serial Communications Controller emulation Copyright by Joakim Larsson Edstrom  
-SuperIO/FDC/FDD/fdimage emulation Copyright by Fred N. van Kempen, Miran Grca, Sarah Walker 2008-2018  
-IDE Emulation Layer Copyright by Alan Cox, 2015  
-DS1202/1302 RTC emulation Copyright by Marco van den Heuvel  
-8250 UART interface and emulation Copyright by smf, Carl  

If you aren't listed and feel you should be, please write to me.

## Other

## License
**GPL**  

z180emu is Copyright Michal Tomek 2018-2019, and others  

```
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
```

See the file COPYING for details.


This program includes software code developed by several people, subject
to the GNU General Public License. Some files are also under the 
BSD 3-clause license. All copyrights are acknowledged to their respective
holders. See individual files for details. 

```
                            NO WARRANTY

BECAUSE THE PROGRAM IS LICENSED FREE OF CHARGE, THERE IS NO WARRANTY
FOR THE PROGRAM, TO THE EXTENT PERMITTED BY APPLICABLE LAW.  EXCEPT WHEN
OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES
PROVIDE THE PROGRAM "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED
OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE ENTIRE RISK AS
TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS WITH YOU.  SHOULD THE
PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING,
REPAIR OR CORRECTION.

IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING
WILL ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY MODIFY AND/OR
REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES,
INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING
OUT OF THE USE OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED
TO LOSS OF DATA OR DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY
YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER
PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES.
```
