Digital Piano
---

The digital piano is a piano system based on the Altera DE2-115 FPGA Board.

* Functionality
	* Sound synthesis
		* Sounds are synthesized using the Audio CODEC on the DE2-115 FPGA as square waves. Since the keyboard has only one octave, the initial octave starts at middle C on a piano and could move an octave up or down by pressing buttons on the FPGA.
	* Play Along 
		* Users could choose to enter the Play Along mode. In Play Along mode, the VGA would cue the user of the next note to play and would stay that way until the note is pressed. Currently, the only built-in song is a C major scale.
	* VGA
		* A keyboard shown in VGA helps the user better visualize the keyboard.
![VGA](/image/vga.png)
	* Read and write from SD Card
		* The user could choose to write the music they are going to play into an SD Card and later reads and plays from the contents of the SD Card.
* Self Assembled Piano Keyboard
	* Circuit
		* Pull-up-pull-down networks using push buttons to serve as keys
	* 3D printed keys
		* 3D printed keys using 3D modeling in SolidWorks
![Keyboard](/image/keyboard.png)
* Implementation
	* SD Card
		* 2GB SDC Card
		* Initialized using sequence of commands CMD0 + CMD8 + CMD1
		* Modified SD controller found in http://web.mit.edu/6.111/www/f2017/tools/sd_controller.v
		* Read and write operations achieved using on-chip SRAM buffers
	* Processor Integration
		* Harmony and chords are calculated using the ECE 350 processor that runs assembly code in a loop to calculate the results
		* Assembly code:
![Assembly](/image/assembly.png)