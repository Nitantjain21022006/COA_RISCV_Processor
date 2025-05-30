# 🔧 RISC Processor Design and Simulation  

This project involves the design and simulation of a five-cycle non-pipelined 25-bit RISC (Reduced Instruction Set Computer) processor using Verilog HDL. Developed as part of **CSE302: Computer Organization and Architecture**, this project implements the core architecture, memory handling, and testbench simulation of a simplified RISC processor.

---

## 📌 Overview  

The processor was developed and tested using Icarus Verilog and supports a small instruction set with custom memory initialization. It includes:

- A modular processor design (`processor.v`, `subcomponents.v`)  
- A custom testbench for simulation (`RISC_tb.v`)  
- Waveform generation (`Wavedump.vcd`) for debugging and analysis  
- Memory file input (`memfile.txt`) for program loading  
- A batch script (`RunIcarus.bat`) for automating the compilation and simulation process  

---

## 🛠️ Key Components  

### 🔲 Core Files  
- **processor.v** – Main processor module integrating control and datapath  
- **subcomponents.v** – Supporting modules (e.g., ALU, control unit, register file)  
- **RISC_tb.v** – Testbench used for simulation  
- **memfile.txt** – Memory file with sample program instructions  
- **Wavedump.vcd** – Output waveform for simulation analysis  
- **RunIcarus.bat** – Script to compile and run using Icarus Verilog  

### 📜 Documentation  
- **handwritten_assembly.pdf** – Scanned document with handwritten assembly code and analysis  

---

## 🚀 How to Run  

### Requirements  
- [Icarus Verilog](http://iverilog.icarus.com/)  
- GTKWave (for waveform viewing)  

### Steps  
1. Make sure Icarus Verilog is installed.  
2. Run `RunIcarus.bat` (on Windows) to compile and simulate.  
3. Use GTKWave to open `Wavedump.vcd` and inspect signals.  

---

## 📚 Reference  

This project was developed as part of academic coursework under the guidance of faculty from **Ahmedabad University**.  
For deeper understanding, review the handwritten document and Verilog files provided.  

---

## 📃 License  

This project is for academic use only.

---
