/* Debug Definitions */
//`define DEBUG
//`define INSTRUCTION
//`define EXEC
//`define LD_CHANGE

/*Table Write Definitions */
//`define WRITE_TABLE
`define READ_TABLE
//`define WRITE_BLANK

`define ZERO_LOAD 6'b111111
`define ZERO_MEM_LOAD (0 | 63'b11 << 61)
`define LD_LOAD (0 | 63'b1 << 62)

`define INVALID 0
`define ADD 1
`define ORX 2
`define ADDI 3
`define BX 4
`define BC 5
`define BCLR 6
`define LD 7
`define LDU 8
`define MTSPR 9
`define MFSPR 10
`define MTCRF 11
`define STD 12
`define SC 13

`define MAX_QUEUE 10
`define MAX_INSTR 4

`define MAX_MEM_READ 1
`define MAX_READ 2
`define MAX_WRITE 2
`define MAX_MEM_WRITE 1

// branch prediction
`define NUM_TABLES 3 // one for each type of branch (b, bc, bclr)
`define PREDICTION_TABLE_ACC 8 // the amount of bits allowed for storage
`define PREDICTION_TABLE_W 3 // slots in the table
`define PREDICTION_TABLE_H 10 // the weight history

module main();

/*
Note: The ascii art is here because I had a very hard time finding my way through the code
and needed a way to quickly identify what section I was in.  Please enjoy!
               .               ..       .       .   .             .
 .      .     T h i s   i s   t h e   g a l a x y   o f   . . .             .
                     .              .       .                    .      .
.        .               .       .     .            .
   .           .        .                     .        .            .
             .               .    .          .              .   .         .
               _________________      ____         __________
 .       .    /                 |    /    \    .  |          \
     .       /    ______   _____| . /      \      |    ___    |     .     .
             \    \    |   |       /   /\   \     |   |___>   |
           .  \    \   |   |      /   /__\   \  . |         _/               .
 .     ________>    |  |   | .   /            \   |   |\    \_______    .
      |            /   |   |    /    ______    \  |   | \           |
      |___________/    |___|   /____/      \____\ |___|  \__________|    .
  .     ____    __  . _____   ____      .  __________   .  _________
       \    \  /  \  /    /  /    \       |          \    /         |      .
        \    \/    \/    /  /      \      |    ___    |  /    ______|  .
         \              /  /   /\   \ .   |   |___>   |  \    \
   .      \            /  /   /__\   \    |         _/.   \    \            
           \    /\    /  /            \   |   |\    \______>    |   .
            \  /  \  /  /    ______    \  |   | \              /          .
 .       .   \/    \/  /____/      \____\ |___|  \____________/  LS
                               .                                        .
     .                           .         .               .                 .
*/
    initial begin
        $dumpfile("ppc.vcd");
        $dumpvars(0,main);
    end

    reg haltR = 0;

    wire clk;
    wire halt = haltR;

    clock clock0(halt,clk);

    reg [0:31]condition = 0;
    reg [0:63]link = 0;
    reg [0:63]pc = 0, lastPc = 0;
    reg [0:63]counter = 0;
    reg [0:63]xer = 0;

    reg branch = 0;

    // reg zero cache
    reg [0:63]regZero;

    integer i, m, k, executeNum, hash, hashSum;

    initial begin
        for (i = 0; i < 64; i=i+1) regZero[i] = 0;
    end

    /**************
    LOAD MANAGER
    **************/

    reg [0:8]numMemRead = 0;
    reg [0:8]numRead = 0;
    reg [0:8]numWrite = 0;
    reg [0:8]oldNumWrite = 0;
    reg [0:8]numMemWrite = 0;
    reg [0:8]oldNumMemWrite = 0;

    reg [0:60]readMem[0:(`MAX_MEM_READ - 1)];
    reg [0:4]readReg[0:(`MAX_READ - 1)];
    reg [0:4]writeReg[0:(`MAX_WRITE - 1)];
    reg [0:60]writeMem[0:(`MAX_MEM_WRITE - 1)];
    reg [0:(`MAX_WRITE - 1)]portUsed;
    reg [0:(`MAX_MEM_WRITE - 1)]portMemUsed;

    // last load regs
    reg [0:60]lastMemRead[0:(`MAX_MEM_READ - 1)];
    reg [0:4]lastReadReg[0:(`MAX_READ - 1)];
    reg [0:4]lastWriteReg[0:(`MAX_WRITE - 1)];
    reg [0:63]lastWriteData[0:(`MAX_WRITE - 1)];
    reg [0:60]lastMemWrite[0:(`MAX_MEM_WRITE - 1)];
    reg [0:63]lastMemWriteData[0:(`MAX_MEM_WRITE - 1)];

    // need two back for writing memory due to scheduler bug with yeild focus
    // in this free version of verilog, :0 does not work as intended
    reg historyWriteEn = 0;
    reg [0:60]historyWriteMem;
    reg [0:63]historyWriteData;

    reg portSwap = 0;

    reg [0:63]writeData[0:`MAX_WRITE];
    reg [0:63]writeMemData[0:`MAX_MEM_WRITE];

    /**************
    DECODE REGS
    **************/

    // memory load
    reg [0:62]m0[0:(`MAX_QUEUE - 1)];

    // reg to load
    reg [0:5]r0[0:(`MAX_QUEUE - 1)];
    reg [0:5]r1[0:(`MAX_QUEUE - 1)];

    // reg to write
    reg [0:5]w0[0:(`MAX_QUEUE - 1)];
    reg [0:5]w1[0:(`MAX_QUEUE - 1)];

    // reg to mem write
    reg [0:62]wm0[0:(`MAX_QUEUE - 1)];

    // instruction ID
    reg [0:5]iid[0:(`MAX_QUEUE - 1)];

    // instruction subID
    reg [0:10]siid[0:(`MAX_QUEUE - 1)];

    // instruction data
    reg [0:63]idata[0:(`MAX_QUEUE - 1)];

    // loaded data (not all used at once)
    reg [0:65]inA[0:(`MAX_QUEUE - 1)];
    reg [0:65]inB[0:(`MAX_QUEUE - 1)];

    reg [0:65]mA[0:(`MAX_QUEUE - 1)];

    // store the pc value of this instruction
    reg [0:63] ipc[0:(`MAX_QUEUE - 1)];

    // states whether or not a branch is predicted
    reg [0:64] bp[0:(`MAX_QUEUE - 1)];

    /**************
    QUEUE DATA
    **************/
    reg [0:5]expectedVal = 0;
    reg [0:1]oddCycle = 0;
    reg signed [0:63]queueSize = 0;
    wire [0:63]queueRoom = `MAX_QUEUE - $signed(queueSize) - expectedVal;

    wire [0:63]pcOffset = (numMemRead == 0 ? (queueRoom >= 4 ? 16 : (queueRoom >= 2 ? 8 : 0)) : (queueRoom >= 2 ? 8 : 0));

    /********************/
    /* Memory interface */
    /********************/

    wire [0:60]subPC[0:(`MAX_INSTR - 1)];

    wire [0:1]subWrite[0:(`MAX_INSTR - 1)];

    wire [0:31]instruction[0:(`MAX_INSTR - 1)];

    wire [0:5]opcode[0:(`MAX_INSTR - 1)];
    wire [0:9]extendedOp[0:(`MAX_INSTR - 1)];

    wire [0:4]rD[0:(`MAX_INSTR - 1)];
    wire [0:4]rA[0:(`MAX_INSTR - 1)];
    wire [0:4]rB[0:(`MAX_INSTR - 1)];
    wire [0:(`MAX_INSTR - 1)]OE;
    wire [0:(`MAX_INSTR - 1)]RC;

    wire [3:18]simm[0:(`MAX_INSTR - 1)];

    // wires for branch
    wire [2:15]bLI[0:(`MAX_INSTR - 1)];
    wire [0:(`MAX_INSTR - 1)]bAA;
    wire [0:(`MAX_INSTR - 1)]bLK;
    wire [2:15]bD[0:(`MAX_INSTR - 1)];

    // wires for ld
    wire [0:1]loadExOp[0:(`MAX_INSTR - 1)];
    wire [3:16]loadDS[0:(`MAX_INSTR - 1)];

    // wire for spr
    wire [0:9]spr[0:(`MAX_INSTR - 1)];

    // wire for sc
    wire [0:6]LEV[0:(`MAX_INSTR - 1)];

    genvar j;

    // update all wires for instruction
    for (j = 0; j < 4; j=j+1)
    begin
        assign subPC[j] = (lastPc - expectedVal * 4 + (expectedVal - j - 1) * 4) >> 3;
        assign subWrite[j] = memWriteEn ? (subPC[j] == memWriteAddr) : (historyWriteEn ? (subPC[j] == historyWriteMem) * 2 : 0);

        case (j)
            0:
                assign instruction[j] = subWrite[j] == 1 ? memWriteData[32:63] : (subWrite[j] == 2 ? historyWriteData[32:63] : memReadData1[32:63]);
            1:
                assign instruction[j] = subWrite[j] == 1 ? memWriteData[0:31] : (subWrite[j] == 2 ? historyWriteData[0:31] : memReadData1[0:31]);
            2:
                assign instruction[j] = subWrite[j] == 1 ? memWriteData[32:63] : (subWrite[j] == 2 ? historyWriteData[32:63] : memReadData0[32:63]);
            3:
                assign instruction[j] = subWrite[j] == 1 ? memWriteData[0:31] : (subWrite[j] == 2 ? historyWriteData[0:31] : memReadData0[0:31]);
        endcase

        assign opcode[j] = instruction[j][0:5];
        assign extendedOp[j] = instruction[j][21:30];
        assign rD[j] = instruction[j][6:10];
        assign rA[j] = instruction[j][11:15];
        assign rB[j] = instruction[j][16:20];
        assign OE[j] = instruction[j][21];
        assign RC[j] = instruction[j][31];
        assign simm[j] = instruction[j][16:31];
        assign bLI[j] = instruction[j][6:29];
        assign bAA[j] = instruction[j][30];
        assign bLK[j] = instruction[j][31];
        assign bD[j] = instruction[j][16:29];
        assign loadExOp[j] = instruction[j][30:31];
        assign loadDS[j] = instruction[j][16:29];
        assign spr[j] = instruction[j][11:20];
        assign LEV[j] = instruction[j][20:26];
    end

    /********************/
    /* Load Data       */
    /********************/

    reg [0:31]instructionL;

    reg [0:5]opcodeL;
    reg [0:9]extendedOpL;

    reg [0:4]rDL;
    reg [0:4]rAL;
    reg [0:4]rBL;
    reg OEL;
    reg RCL;

    reg [3:18]simmL;

    // reg for branch
    reg [2:15]bLIL;
    reg bAAL;
    reg bLKL;
    reg [2:15]bDL;

    // reg for ld
    reg [0:1]loadExOpL;
    reg [3:16]loadDSL;

    // reg for spr
    reg [0:9]sprL;

    // reg for sc
    reg [0:6]LEVL;

    /********************/
    /* Memory interface */
    /********************/

    wire memReadEn0 = 1;
    wire [0:60]memReadAddr0 = pcOffset < 12 ? lastMemRead[0] : lastPc[0:60];
    wire [0:63]memReadData0;

    wire memReadEn1 = pcOffset != 0;
    wire [0:60]memReadAddr1 = pcOffset < 12 ? lastPc[0:60] : lastPc[0:60] + 1;
    wire [0:63]memReadData1;

    wire memWriteEn = oldNumMemWrite != 0;
    wire [3:63]memWriteAddr = lastMemWrite[0];
    wire [0:63]memWriteData = lastMemWriteData[0];

    mem mem0(clk,
        memReadEn0,memReadAddr0,memReadData0,
        memReadEn1,memReadAddr1,memReadData1,
        memWriteEn,memWriteAddr,memWriteData);

    /********/
    /* regs */
    /********/

    wire regReadEn0 = 1;
    wire [0:4]regReadAddr0 = lastReadReg[0];
    wire [0:63]regReadData0;

    wire regReadEn1 = 1;
    wire [0:4]regReadAddr1 = lastReadReg[1];
    wire [0:63]regReadData1;

    wire regWriteEn0 = !portSwap ? oldNumWrite > $signed(0) : 0;
    wire [0:4]regWriteAddr0 = lastWriteReg[0];
    wire [0:63]regWriteData0 = lastWriteData[0];

    wire regWriteEn1 = !portSwap ? oldNumWrite > $signed(1) : 1;
    wire [0:4]regWriteAddr1 = lastWriteReg[1];
    wire [0:63]regWriteData1 = lastWriteData[1];

    regs gprs(clk,
       /* Read port #0 */
       regReadEn0,
       regReadAddr0, 
       regReadData0,

       /* Read port #1 */
       regReadEn1,
       regReadAddr1, 
       regReadData1,

       /* Write port #0 */
       regWriteEn0,
       regWriteAddr0, 
       regWriteData0,

       /* Write port #1 */
       regWriteEn1,
       regWriteAddr1, 
       regWriteData1
    );

    /**************
    BRANCH PREDICTION (WARNING ALL NIGHTER DETECTED)
    * Note: my form of branch prediction is based off of linear trends.
    * The algorithm learns based on the data it is presented.  After each test exits,
    * the data is stored in an output file and loaded when the next test starts.
    * The startup data comes from the output of the tests after being run for training
    * on the p9 released test set.
    **************/
    reg signed [0: (`PREDICTION_TABLE_ACC - 1)]predictionTable [0 : (`PREDICTION_TABLE_W * `PREDICTION_TABLE_H - 1) * `NUM_TABLES];
    reg signed [0:(`PREDICTION_TABLE_W - 1)]predictionHistory [0:(`NUM_TABLES - 1)];

    // setup the prediction table
    initial begin

`ifndef READ_TABLE
    for (i = 0; i < `PREDICTION_TABLE_W * `PREDICTION_TABLE_H * `NUM_TABLES; i=i+1)
    begin
        predictionTable[i] = 0;
    end

    for (i = 0; i < `NUM_TABLES; i=i+1)
    begin
        predictionHistory[i] = 0;
    end

`ifdef WRITE_BLANK
    // output the table file
    $writememh("table.txt", predictionTable);

    // output the history file
    $writememh("history.txt", predictionHistory);
`endif

`endif

`ifdef READ_TABLE
    $readmemh("./table.txt", predictionTable);

    $readmemh("./history.txt", predictionHistory);
`endif
    end

    /**************
    MISCELLANEOUS
    **************/

    integer offset;
    integer lock = 0, lock2 = 0;

    /**************
    Start of the Blocking Section
           __
.-.__      \ .-.  ___  __
|_|  '--.-.-(   \/\;;\_\.-._______.-.
(-)___     \ \ .-\ \;;\(   \       \ \
 Y    '---._\_((Q)) \;;\\ .-\     __(_)
 I           __'-' / .--.((Q))---'    \,
 I     ___.-:    \|  |   \'-'_          \
 A  .-'      \ .-.\   \   \ \ '--.__     '\
 |  |____.----((Q))\   \__|--\_      \     '
    ( )        '-'  \_  :  \-' '--.___\
     Y                \  \  \       \(_)
     I                 \  \  \         \,
     I                  \  \  \          \
     A                   \  \  \          '\
     |                    \  \__|           '
                           \_:.  \
                             \ \  \
                              \ \  \
                               \_\_|
    **************/

    // decode all instructions (get what is needed from load)
    always @ (posedge clk)
    begin

`ifdef DEBUG
    $display("MEMORY: %d, %d, %d, %d", regWriteAddr0, regWriteAddr1, regReadAddr0, regReadAddr1);
    $display("MEMORY: %d, %d, %d, %d", regWriteData0, regWriteData1, regReadData0, regReadData1);
    $display("EN: %d, %d, %d, %d", regWriteEn0, regWriteEn1, regReadEn0, regReadEn1);
`endif
        // set port used to false before doing anything
        for (i = 0; i < `MAX_WRITE; i = i + 1)
        begin
            portUsed[i] = 0;

            portMemUsed[i] = 0;
        end

        /**************
        UPDATE QUEUE MANAGER
           ________________
         |'-.--._ _________:
         |  /    |  __    __\
         | |  _  | [\_\= [\_\
         | |.' '. \.........|
         | ( <)  ||:       :|_
          \ '._.' | :.....: |_(o
           '-\_   \ .------./
           _   \   ||.---.||  _
          / \  '-._|/\n~~\n' | \
         (| []=.--[===[()]===[) |
         <\_/  \_______/ _.' /_/
         ///            (_/_/
         |\\            [\\
         ||:|           | I|
         |::|           | I|
         ||:|           | I|
         ||:|           : \:
         |\:|            \I|
         :/\:            ([])
         ([])             [|
          ||              |\_
         _/_\_            [ -'-.__
        <]   \>            \_____.>
          \__/
        **************/

        offset = 0;

        // shift everything over by the expectedVal
        for (i = queueSize + expectedVal - 1; $signed(expectedVal) <= i; i=i-1)
        begin
            if (iid[i - expectedVal] != `INVALID)
            begin
                // memory load
                m0[i] = m0[i - expectedVal];

                // reg to load
                r0[i] = r0[i - expectedVal];
                r1[i] = r1[i - expectedVal];

                // reg to write
                w0[i] = w0[i - expectedVal];
                w1[i] = w1[i - expectedVal];

                // reg to memory write
                wm0[i] = wm0[i - expectedVal];

                // the ipc
                ipc[i] = ipc[i - expectedVal];

                // instruction ID
                iid[i] = iid[i - expectedVal];

                // instruction subID
                siid[i] = siid[i - expectedVal];

                // instruction data
                idata[i] = idata[i - expectedVal];

                // loaded data (not all used at once)
                inA[i] = inA[i - expectedVal];
                inB[i] = inB[i - expectedVal];

                mA[i] = mA[i - expectedVal];

                // branch prediction
                bp[i] = bp[i - expectedVal];
            end
            else
            begin
                // shift everything to the right over by one to the left
                for (m = i + 1; m < queueSize - offset + expectedVal; m=m+1)
                begin
                    // memory load
                    m0[m - 1] = m0[m];

                    // reg to load
                    r0[m - 1] = r0[m];
                    r1[m - 1] = r1[m];

                    // reg to write
                    w0[m - 1] = w0[m];
                    w1[m - 1] = w1[m];

                    // reg to memory write
                    wm0[m - 1] = wm0[m];

                    // the ipc
                    ipc[m - 1] = ipc[m];

                    // instruction ID
                    iid[m - 1] = iid[m];

                    // instruction subID
                    siid[m - 1] = siid[m];

                    // instruction data
                    idata[m - 1] = idata[m];

                    // loaded data (not all used at once)
                    inA[m - 1] = inA[m];
                    inB[m - 1] = inB[m];

                    mA[m - 1] = mA[m];

                    // branch prediction
                    bp[m - 1] = bp[m];
                end

                // update offset
                offset = offset + 1;
            end
        end

        // update the next queue size
        queueSize = queueSize - offset + expectedVal;

        /**************
        DECODE MANAGER
                              __.|_nO_/\__I_(D___
                       __,---'...:..:..:..:..:...`---.__
                 __,--'  .'    :   :   :   :   :    `.  `--.__
           __,--' -----.'----.'---:----:----:---`.----`.----- `--.__
     __,--'__,_______,'_____,'____:____:____:____`._____`._______.__`--.__
 _,-'_____________________________________________________________________`-._
/_____________________________________________________________________________\
\_____________________________________________________________________________/
         `----.___  ```...     `.      :      ,'     ...'''  ___,----'
                  `---.___``.   `.     :     ,'   .''___,---'
                          `-.`___`.____:____,'___',-'
                                 ):::::::::::(
                                   ):::::::(
                                    `:''':'
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      | |
                                      )-(
                                     (:::)
        **************/

        // no branch here mam
        branch = 0;

        for (i = oddCycle != 1 ? expectedVal - 1 : expectedVal - 2; $signed(0) <= i && !branch; i=i-1)
        begin
            // default setup
            iid[i] = `INVALID;

            m0[i] = `ZERO_MEM_LOAD;

            r0[i] = `ZERO_LOAD;
            r1[i] = `ZERO_LOAD;
    
            w0[i] = `ZERO_LOAD;
            w1[i] = `ZERO_LOAD;

            wm0[i] = `ZERO_MEM_LOAD;

            ipc[i] = pc - expectedVal * 4 + (expectedVal - i - 1) * 4;

            bp[i] = 0;

            // set the load data
            inA[i] = 0;
            inB[i] = 0;
            mA[i] = 0;

            // decode
            case (opcode[i])
                31:
                begin
                    if (extendedOp[i][1:9] == 266) // add
                    begin
`ifdef INSTRUCTION
    $display("ADD %d, %d %d", rD[i], rA[i], rB[i]);
`endif
                        iid[i] = `ADD;

                        m0[i] = `ZERO_MEM_LOAD;

                        r0[i] = regZero[rA[i]] ? {1'b0, rA[i]} : `ZERO_LOAD;
                        r1[i] = regZero[rB[i]] ? {1'b0, rB[i]} : `ZERO_LOAD;
    
                        w0[i] = {1'b0, rD[i]};
                        w1[i] = `ZERO_LOAD;

                        siid[i] = 0 | RC[i] << 1 | OE[i];
                    end
                    else if (extendedOp[i] == 444) // orx
                    begin
`ifdef INSTRUCTION
    $display("ORX");
`endif
                        iid[i] = `ORX;

                        r0[i] = regZero[rD[i]] ? {1'b0, rD[i]} : `ZERO_LOAD;
                        r1[i] = regZero[rB[i]] ? {1'b0, rB[i]} : `ZERO_LOAD;

                        w0[i] = {1'b0, rA[i]};
                        w1[i] = `ZERO_LOAD;

                        // store OE in sub id
                        siid[i] = 0 | RC[i];
                    end
                    else if (extendedOp[i] == 144) // mtcrf
                    begin
`ifdef INSTRUCTION
    $display("MTCRF");
`endif
                        if (!spr[i][0])
                        begin
                            iid[i] = `MTCRF;

                            r0[i] = regZero[rD[i]] ? {1'b0, rD[i]} : `ZERO_LOAD;
                            r1[i] = `ZERO_LOAD;

                            idata[i] = { {4{spr[i][1]}}, {4{spr[i][2]}}, {4{spr[i][3]}}, {4{spr[i][4]}}, {4{spr[i][5]}}, {4{spr[i][6]}}, {4{spr[i][7]}}, {4{spr[i][8]}} };
                        end
                    end
                    else if (extendedOp[i] == 467) // mtspr
                    begin
`ifdef INSTRUCTION
    $display("MTSPR");
`endif
                        if ((spr[i][0:4] == 5'b00001 || spr[i][0:4] == 5'b01000 || spr[i][0:4] == 5'b01001) && spr[i][5:9] == 0)
                        begin
                            iid[i] = `MTSPR;

                            r0[i] = regZero[rD[i]] ? {1'b0, rD[i]} : `ZERO_LOAD;
                            r1[i] = `ZERO_LOAD;

                            idata[i] = spr[i][0:4];
                        end
                     end
                     else if (extendedOp[i] == 339) // mfspr
                     begin
`ifdef INSTRUCTION
    $display("MFSPR");
`endif
                         if ((spr[i][0:4] == 5'b00001 || spr[i][0:4] == 5'b01000 || spr[i][0:4] == 5'b01001) && spr[i][5:9] == 0)
                         begin
                             iid[i] = `MFSPR;

                             w0[i] = rD[i];
                             w1[i] = `ZERO_LOAD;

                             idata[i] = spr[i][0:4];
                         end
                    end
                end
                14: // addi
                begin
`ifdef INSTRUCTION
    $display("ADDI %d, %d, %d", rD[i], rA[i], { {48{simm[i][3]}}, simm[i] });
`endif
                    iid[i] = `ADDI;

                    m0[i] = `ZERO_MEM_LOAD;

                    if (rA[i] == 0)
                        r0[i] = `ZERO_LOAD;
                    else
                        r0[i] = regZero[rA[i]] ? {1'b0, rA[i]} : `ZERO_LOAD;

                    r1[i] = `ZERO_LOAD;

                    w0[i] = {1'b0, rD[i]};
                    w1[i] = `ZERO_LOAD;

                    // store OE in sub id
                    idata[i] = { {48{simm[i][3]}}, simm[i] };
                end
                18: // bx (this is the only instruction that is evaluated immediately)
                begin
`ifdef INSTRUCTION
    $display("BX");
`endif
                    iid[i] = `BX;

                    if (bAA[i])
                        idata[i] = { {48{bLI[i][2]}}, bLI[i], 2'b00 };
                    else
                        idata[i] = ipc[i] + { {48{bLI[i][2]}}, bLI[i], 2'b00 };

                    // store bLK, rD, and rA in sub id (only bLK is needed)
                    siid[i] = 0 | bLK[i];

                    // evaluate the branch
                    bp[i] = 1;
                end
                16: // bc
                begin
`ifdef INSTRUCTION
    $display("BC");
`endif
                    iid[i] = `BC;

                    // store bLK, rD, and rA in sub id (only bLK is needed)
                    siid[i] = 0 | rA[i] << 6 | rD[i] << 1 | bLK[i];

                    // store the immediate
                    if (bAA[i])
                        idata[i] = { {48{bD[i][2]}}, bD[i], 2'b00 };
                    else
                        idata[i] = ipc[i] + { {48{bD[i][2]}}, bD[i], 2'b00 };

                    // evaluate the branch
                    bp[i] = 1;
                end
                19: // bclr
                begin
`ifdef INSTRUCTION
    $display("BCLR");
`endif
                    if (extendedOp[i] == 16)
                    begin
                        iid[i] = `BCLR;

                        // store bLK, rD, and rA in sub id
                        siid[i] = 0 | rA[i] << 6 | rD[i] << 1 | bLK[i];

                        // evaluate the branch
                        bp[i] = 1;
                    end
                end
                58: // ld
                begin
                    case (loadExOp[i])
                        2'b00: // ld
                        begin
`ifdef INSTRUCTION
    $display("LD %d, %d", rD[i], rA[i], $signed({ {48{loadDS[i][3]}}, loadDS[i], 2'b0 }));
`endif
                            iid[i] = `LD;

                            m0[i] = `LD_LOAD;

                            r0[i] = (rA[i] != 0 && regZero[rA[i]]) ? {1'b0, rA[i]} : `ZERO_LOAD;
                            r1[i] = `ZERO_LOAD;

                            w0[i] = {1'b0, rD[i]};
                            w1[i] = `ZERO_LOAD;

                            idata[i] = { {48{loadDS[i][3]}}, loadDS[i], 2'b0 };
                        end
                        2'b01: // ldu
                        begin
`ifdef INSTRUCTION
    $display("LDU");
`endif
                            if (rA[i] != 0 && rA[i] != rD[i]) // check the instruction is valid
                            begin
                                iid[i] = `LDU;

                                m0[i] = `LD_LOAD;

                                r0[i] = regZero[rA[i]] ? {1'b0, rA[i]} : `ZERO_LOAD;
                                r1[i] = `ZERO_LOAD;

                                w0[i] = {1'b0, rD[i]};
                                w1[i] = {1'b0, rA[i]};

                                idata[i] = { {48{loadDS[i][3]}}, loadDS[i], 2'b0 };
                            end
                        end
                       
                    endcase
                end
                62: // std
                begin
`ifdef INSTRUCTION
    $display("STD");
`endif
                    if (loadExOp[i] == 0)
                    begin
                        iid[i] = `STD;

                        r0[i] = regZero[rD[i]] ? {1'b0, rD[i]} : `ZERO_LOAD;
                        r1[i] = (rA[i] != 0 && regZero[rA[i]]) ? {1'b0, rA[i]} : `ZERO_LOAD;

                        idata[i] = { {48{loadDS[i][3]}}, loadDS[i], 2'b0 };

                        // turn off regzero optimization as pipeline code may change
                        for (m = 0; m < 64; m = m + 1)
                        begin
                            regZero[m] = 1;
                        end
                    end
                end
                17: // sc
                begin
                    if (bAA[i] && LEV[i][0:5] == 0)
                    begin
`ifdef INSTRUCTION
    $display("SC");
`endif
                        iid[i] = `SC;

                        m0[i] = `ZERO_MEM_LOAD;

                        r0[i] = regZero[0] ? {1'b0, 4'b0000} : `ZERO_LOAD;
                        r1[i] = regZero[3] ? {1'b0, 4'b0011} : `ZERO_LOAD;

                        w0[i] = `ZERO_LOAD;
                        w1[i] = `ZERO_LOAD;
                    end
                end
            endcase

           // update zero load
           if (w0[i] != `ZERO_LOAD)
               regZero[w0[i][1:5]] = 1'b1;

           if (w1[i] != `ZERO_LOAD)
               regZero[w1[i][1:5]] = 1'b1;

            /**************
            BRANCH PREDICTION (IT'S A TRAP!)
                            __...------------._
                         ,-'                   `-.
                      ,-'                         `.
                    ,'                            ,-`.
                   ;                              `-' `.
                  ;                                 .-. \
                 ;                           .-.    `-'  \
                ;                            `-'          \
               ;                                          `.
               ;                                           :
              ;                                            |
             ;                                             ;
            ;                            ___              ;
           ;                        ,-;-','.`.__          |
       _..;                      ,-' ;`,'.`,'.--`.        |
      ///;           ,-'   `. ,-'   ;` ;`,','_.--=:      /
     |'':          ,'        :     ;` ;,;,,-'_.-._`.   ,'
     '  :         ;_.-.      `.    :' ;;;'.ee.    \|  /
      \.'    _..-'/8o. `.     :    :! ' ':8888)   || /
       ||`-''    \\88o\ :     :    :! :  :`""'    ;;/
       ||         \"88o\;     `.    \ `. `.      ;,'
       /)   ___    `."'/(--.._ `.    `.`.  `-..-' ;--.
       \(.="""""==.. `'-'     `.|      `-`-..__.-' `. `.
        |          `"==.__      )                    )  ;
        |   ||           `"=== '                   .'  .'
        /\,,||||  | |           \                .'   .'
        | |||'|' |'|'           \|             .'   _.' \
        | |\' |  |           || ||           .'    .'    \
        ' | \ ' |'  .   ``-- `| ||         .'    .'       \
          '  |  ' |  .    ``-.._ |  ;    .'    .'          `.
       _.--,;`.       .  --  ...._,'   .'    .'              `.__
     ,'  ,';   `.     .   --..__..--'.'    .'                __/_\
   ,'   ; ;     |    .   --..__.._.'     .'                ,'     `.
  /    ; :     ;     .    -.. _.'     _.'                 /         `
 /     :  `-._ |    .    _.--'     _.'                   |
/       `.    `--....--''       _.'                      |
          `._              _..-'                         |
             `-..____...-''                              |
                                                         |
                                                         |
            **************/

            if (bp[i] == 1)
            begin
                // hash the number (Knuth Algorithm)
                hash = (ipc[i] * 2654435761 >> 32) % `PREDICTION_TABLE_H;

                // starting value for sum
                hashSum = predictionTable[(iid[i] - `BX) * `PREDICTION_TABLE_W * `PREDICTION_TABLE_H + hash * `PREDICTION_TABLE_W];

                // sum up the weights
                for (m = 1; m < `PREDICTION_TABLE_W; m=m+1)
                    hashSum = hashSum + predictionTable[(iid[i] - `BX) * `PREDICTION_TABLE_W * `PREDICTION_TABLE_H + hash * `PREDICTION_TABLE_W + m] * (predictionHistory[(iid[i] - `BX)][m] ? 1 : -1);

                if (hashSum < $signed(0))
                    bp[i] = 0; // not taken
                else
                begin
                    // check the type of branch to take
                    case (iid[i])
                        `BX:
                         begin
                             bp[i] = 1;

                             bp[i] = idata[i]; // store the branch location

                             pc = idata[i];
                         end
                        `BC:
                         begin
                             bp[i] = 1;

                             bp[i] = idata[i]; // store the branch location

                             pc = idata[i];
                         end
                        `BCLR:
                         begin
                             bp[i] = 1;

                             bp[i] = link; // store the branch location

                             pc = link;
                         end
                    endcase

                    bp[i][0] = 1; // taken

                    branch = 1;
                end
            end
        end

        // remove instruction based on odd offset
        if (oddCycle == 1)
        begin
            iid[expectedVal - 1] = `INVALID;

            m0[expectedVal - 1] = `ZERO_MEM_LOAD;

            r0[expectedVal - 1] = `ZERO_LOAD;
            r1[expectedVal - 1] = `ZERO_LOAD;
    
            w0[expectedVal - 1] = `ZERO_LOAD;
            w1[expectedVal - 1] = `ZERO_LOAD;

            wm0[expectedVal - 1] = `ZERO_MEM_LOAD;

            ipc[expectedVal - 1] = 0;

            bp[expectedVal - 1] = 0;
        end

        // destroy instructions based on the branch
        for (i = i; $signed(0) <= i && branch; i=i-1)
        begin
            iid[i] = `INVALID;

            m0[i] = `ZERO_MEM_LOAD;

            r0[i] = `ZERO_LOAD;
            r1[i] = `ZERO_LOAD;
    
            w0[i] = `ZERO_LOAD;
            w1[i] = `ZERO_LOAD;

            wm0[i] = `ZERO_MEM_LOAD;

            ipc[i] = 0;

            bp[i] = 0;
        end

`ifdef DEBUG
$display("---------------");
        for (m = 0; m < queueSize; m=m+1)
        begin
$display("ID: %d: %d, OA: %d, IA: %d, IB: %d, IM: %d, A: %b, B: %b, M: %d, R: %d, W: %d", m, iid[m], w0[m][1:5], r0[m], r1[m], m0[m], inA[m], inB[m], mA[m][2:65], writeReg[1], executeNum);
        end
$display("---------------***");
`endif

        /**************
        UPDATE INFO QUEUE MANAGER
        LOAD DATA INTO ANYTHING WITH 1, 0
                ___      |\________/)
               [_,_])    \ /       \|
              /|=T=|]     /   __  __\
              |\ " //     |_  9   p ]\
              ||'-'/--.  / /\ =|  \|\ \
             /|| <\/> |\ | '._, @ @)<_)
            | |\   |  |   \.__/(_;_)
            |  .   H  |   |  :  '='|
            |  |  _H__/  _| :      |
             \  '.__  \ /  ;      ';
            __'-._(_}==.'  :       ;
           (___|    /-' |   :.     :
          [.-'  \   |   \   \ ;   :
         .-'     |  |    |  |   ":
        /        |==|     \  \  /  \_
       /         [  |      '._\_ -._ \
      /           \__)   __.- \ \   )\\
     /       |        /.'      >>)
     |        \       |\     |
     |     .'  '-.    | \    /
     |    /     /     / /   /
                |    / 
        **************/

        for (i = queueSize - 1; $signed(0) <= i; i=i-1)
        begin
            // MEMORY A

            if (mA[i][0] == 1'b1 && mA[i][1] == 1'b0)
            begin
               if (mA[i][2] == 1)
                   mA[i][2] = 0;
               else if (mA[i][2:65] == 0)
               begin
                   mA[i] = {1'b0, 1'b1, memReadData0};
               end
               else if (mA[i][2:65] == 1)
               begin
                   mA[i] = {1'b0, 1'b1, memReadData1};
               end
            end

            // REG A

            if (inA[i][0] == 1'b1 && inA[i][1] == 1'b0)
            begin
               if (inA[i][2] == 1)
                   inA[i][2] = 0;
               else if (inA[i][2:65] == 0)
               begin
                   inA[i] = {1'b0, 1'b1, regReadData0};
               end
               else if (inA[i][2:65] == 1)
               begin
                   inA[i] = {1'b0, 1'b1, regReadData1};
               end
            end

            // REG B

            if (inB[i][0] == 1'b1 && inB[i][1] == 1'b0)
            begin

               if (inB[i][2] == 1)
                   inB[i][2] = 0;
               else if (inB[i][2:65] == 0)
               begin
                   inB[i] = {1'b0, 1'b1, regReadData0};
               end
               else if (inB[i][2:65] == 1)
               begin
                   inB[i] = {1'b0, 1'b1, regReadData1};
               end
            end
        end

        /**************
        LOAD QUEUE MANAGER
        0, 0 means unevaluated
        0, 1 means loaded
        1, 0 means loading
        1, 1 means another instruction outputs the data
          ,-'//__\\`-.          
        ,'  ____      `.        
       /   / ,-.-.      \       
      (/# /__`-'_| || || )      
      ||# []/()] O || || |      
    __`------------------'__    
   |--| |<=={_______}=|| |--|   
   |  | |-------------|| |  |   
   |  | |={_______}==>|| |  |   
   |  | |   |: _ :|   || |  |   
   > _| |___|:===:|   || |__<   
   :| | __| |: - :|   || | |:   
   :| | ==| |: _ :|   || | |:   
   :| | ==|_|:===:|___||_| |:   
   :| |___|_|:___:|___||_| |:   
   :| |||   ||/_\|| ||| -| |:   
   ;I_|||[]_||\_/|| ||| -|_I;   
   |_ |__________________| _|   
   | `\\\___|____|____/_//' |   
   J : |     \____/     | : L   
  _|_: |      |__|      | :_|_  
-/ _-_.'    -/    \-    `.-_- \-
/______\    /______\    /______\
        **************/

        // rest reads and writes
        numMemRead = 0;
        numRead = 0;
        numWrite = 0;

        // clear out all loaded wm0's
        for (i = queueSize - 1; 0 <= i; i=i-1)
        begin
            wm0[i] = `ZERO_MEM_LOAD;
        end

        // load as much as possible per cycle
        for (i = queueSize - 1; 0 <= i; i=i-1)
        begin
            // UPDATE LD's

            if (m0[i] == `LD_LOAD && (r0[i] == `ZERO_LOAD || (inA[i][0] == 1'b0 && inA[i][1] == 1'b1)))
            begin
                // store update in idata due to the possibility the data is not alligned
                idata[i] = idata[i] + (r0[i] == `ZERO_LOAD ? 0 : inA[i][2:65]);

                // update the data for the address
                m0[i] = idata[i][0:60];
            end

            // MEMORY A

            lock = 0;

            if (m0[i] != `ZERO_MEM_LOAD && m0[i] != `LD_LOAD && (mA[i][0] == 1'b0 && mA[i][1] == 1'b0) && lock == 0)
            begin
                // check if the data comes from an instruction that modifies memory
                for (m = i + 1; m < queueSize; m=m+1)
                begin
                    if (m0[i] == wm0[m][2:62] && wm0[m][0] == 1'b0 && wm0[m][1] == 1'b1)
                    begin
                        mA[i] = inA[m];

                        lock = 1;
                    end
                end

                // check if the data comes from memory being modified
                if (memWriteEn && memWriteAddr == m0[i] && lock == 0)
                begin
                    mA[i] = {1'b0, 1'b1, memWriteData};
                    lock = 1;
                end

                if (historyWriteEn && historyWriteMem == m0[i] && lock == 0)
                begin
                    mA[i] = {1'b0, 1'b1, historyWriteData};
                    lock = 1;
                end

                // check if the data is loaded
                for (m = i + 1; m < queueSize && lock == 0; m=m+1)
                begin
                    if (m0[i] == m0[m])
                    begin
                        if (mA[m][0] == 1'b0 && mA[m][1] == 1'b1)
                        begin
                            mA[i] = mA[m];
                            lock = 1;
                        end
                    end
                end

                // check if the data is being loaded already
                for (m = 0; m < numMemRead && lock == 0; m = m + 1)
                begin
                    if (m0[i] == readMem[m])
                    begin
                        mA[i] = 3'b101 << 63 | m;

                        lock = 1;
                    end
                end

                // check if the data needs to be loaded
                if (numMemRead < `MAX_MEM_READ && lock == 0)
                begin
                    readMem[numMemRead] = m0[i];

                    // write down where to retreive the data to
                    mA[i] = 3'b101 << 63 | numMemRead;

                    numMemRead = numMemRead + 1;
                end
            end

            // REG INA

            lock = 0;

            if (r0[i] != `ZERO_LOAD && (inA[i][0] == 1'b0 && inA[i][1] == 1'b0))
            begin
                // check if another instruction will output the data we want
                for (m = i + 1; m < queueSize && lock == 0; m=m+1)
                begin
                    if (r0[i] == w0[m] || r0[i] == w1[m])
                    begin
                        inA[i] = 2'b11 << 64;
                        lock = 1;
                    end
                end

                // check if the data is sitting in output
                if (lock == 0)
                begin
                    if (r0[i] == regWriteAddr0)
                    begin
                        inA[i] = {2'b01, regWriteData0};
                        lock = 1;
                    end
                    else if (r0[i] == regWriteAddr1)
                    begin
                        inA[i] = {2'b01, regWriteData1};
                        lock = 1;
                    end
                end

                // check if the data is already in memory
                for (m = i + 1; m < queueSize && lock == 0; m=m+1)
                begin
                    if (r0[i] == r0[m] && inA[m] != 0)
                    begin
                        inA[i] = inA[m];
                        lock = 1;
                    end
                    else if (r0[i] == r1[m] && inB[m] != 0)
                    begin
                        inA[i] = inB[m];
                        lock = 1;
                    end
                end

                // check if the data is being loaded already
                for (m = 0; m < numRead && lock == 0; m = m + 1)
                begin
                    if (r0[i] == readReg[m])
                    begin
                        inA[i] = 3'b101 << 63 | m;
                        lock = 1;
                    end
                end

                // check if the data needs to be loaded
                if (numRead < `MAX_READ && lock == 0)
                begin
                    readReg[numRead] = r0[i];

                    // write down where to retreive the data to
                    inA[i] = 3'b101 << 63 | numRead;

                    numRead = numRead + 1;
                end
            end

            // REG INB

            lock = 0;

            if (r1[i] != `ZERO_LOAD && (inB[i][0] == 1'b0 && inB[i][1] == 1'b0))
            begin
                // check if another instruction will output the data we want
                for (m = i + 1; m < queueSize && lock == 0; m=m+1)
                begin
                    if (r1[i] == w0[m] || r1[i] == w1[m])
                    begin
                        inB[i] = 2'b11 << 64;
                        lock = 1;
                    end
                end

                // check if the data is sitting in output
                if (lock == 0)
                begin
                    if (r1[i] == regWriteAddr0)
                    begin
                        inB[i] = {2'b01, regWriteData0};
                        lock = 1;
                    end
                    else if (r1[i] == regWriteAddr1)
                    begin
                        inB[i] = {2'b01, regWriteData1};
                        lock = 1;
                    end
                end

                // check if the data is already in memory
                for (m = i + 1; m < queueSize && lock == 0; m=m+1)
                begin
                    if (r1[i] == r0[m] && inA[m] != 0)
                    begin
                        inB[i] = inA[m];
                        lock = 1;
                    end
                    else if (r1[i] == r1[m] && inB[m] != 0)
                    begin
                        inB[i] = inB[m];
                        lock = 1;
                    end
                end

                // check if the data is being loaded already
                for (m = 0; m < numRead && lock == 0; m = m + 1)
                begin
                    if (r1[i] == readReg[m])
                    begin
                        inB[i] = 3'b101 << 63 | m;
                        lock = 1;
                    end
                end

                // check if the data needs to be loaded
                if (numRead < `MAX_READ && lock == 0)
                begin
                    readReg[numRead] = r1[i];

                    // write down where to retreive the data to
                    inB[i] = 3'b101 << 63 |  numRead;

                    numRead = numRead + 1;
                end
            end

            // STD Check
            lock = 0;
            lock2 = 0;

            if (iid[i] == `STD && (r0[i] == `ZERO_LOAD || (inA[i][0] == 1'b0 && inA[i][1] == 1'b1))
                && (r1[i] == `ZERO_LOAD || (inB[i][0] == 1'b0 && inB[i][1] == 1'b1))
                && wm0[i] == `ZERO_MEM_LOAD)
            begin
                // set the write mem
                wm0[i] = (idata[i] + (r1[i] == `ZERO_LOAD ? 0 : inB[i][2:65])) >> 3;

                wm0[i] = {2'b01, wm0[i][2:62]};

                // check if any data in the queue is modified by this instruction
                for (m = i - 1; $signed(0) <= m && lock2 == 0; m = m - 1)
                begin
                    if (ipc[m][0:60] == wm0[i][2:62]) // this data will be modified by ld
                    begin
                        // set the instruction load so the wires can update
                        instructionL = ipc[m][61] ? inA[i][34:65] : inA[i][2:33];

                        // setup data for eval
                        opcodeL = instructionL[0:5];
                        extendedOpL = instructionL[21:30];
                        rDL = instructionL[6:10];
                        rAL = instructionL[11:15];
                        rBL = instructionL[16:20];
                        OEL = instructionL[21];
                        RCL = instructionL[31];
                        simmL = instructionL[16:31];
                        bLIL = instructionL[6:29];
                        bAAL = instructionL[30];
                        bLKL = instructionL[31];
                        bDL = instructionL[16:29];
                        loadExOpL = instructionL[30:31];
                        loadDSL = instructionL[16:29];
                        sprL = instructionL[11:20];
                        LEVL = instructionL[20:26];

                        // undo branch prediction
                        if (bp[m])
                        begin
                            pc = ipc[m] + 4;
                            branch = 1;
                            lock2 = 1;
                        end

                        // set the lock
                        lock = 1;

                        // re-evaluate this data with the new information
                        iid[m] = `INVALID;
            
                        m0[m] = `ZERO_LOAD;
            
                        r0[m] = `ZERO_LOAD;
                        r1[m] = `ZERO_LOAD;
                
                        w0[m] = `ZERO_LOAD;
                        w1[m] = `ZERO_LOAD;

                        wm0[m] = `ZERO_MEM_LOAD;

                        bp[m] = 0;
            
                        // don't update the pc here
            
                        // set the load data
                        inA[m] = 0;
                        inB[m] = 0;
                        mA[m] = 0;
            
                        // decode
                        case (opcodeL)
                            31:
                            begin
                                if (extendedOpL[1:9] == 266) // add
                                begin
`ifdef LD_CHANGE
    $display("L ADD %d, %d %d", rDL, rAL, rBL);
`endif
                                    iid[m] = `ADD;
            
                                    m0[m] = `ZERO_LOAD;
            
                                    r0[m] = regZero[rAL] ? {1'b0, rAL} : `ZERO_LOAD;
                                    r1[m] = regZero[rBL] ? {1'b0, rBL} : `ZERO_LOAD;
                
                                    w0[m] = {1'b0, rDL};
                                    w1[m] = `ZERO_LOAD;
            
                                    siid[m] = 0 | RCL << 1 | OEL;
                                end
                                else if (extendedOpL == 444) // orx
                                begin
`ifdef LD_CHANGE
    $display("L ORX");
`endif
                                    iid[m] = `ORX;
            
                                    r0[m] = regZero[rDL] ? {1'b0, rDL} : `ZERO_LOAD;
                                    r1[m] = regZero[rBL] ? {1'b0, rBL} : `ZERO_LOAD;
            
                                    w0[m] = {1'b0, rAL};
                                    w1[m] = `ZERO_LOAD;
            
                                    // store OE in sub id
                                    siid[m] = 0 | RCL;
                                end
                                else if (extendedOpL == 144) // mtcrf
                                begin
`ifdef LD_CHANGE
    $display("L MTCRF");
`endif
                                    if (!sprL[m][0])
                                    begin
                                        iid[m] = `MTCRF;
            
                                        r0[m] = regZero[rDL] ? {1'b0, rDL} : `ZERO_LOAD;
                                        r1[m] = `ZERO_LOAD;
            
                                        idata[m] = { {4{sprL[1]}}, {4{sprL[2]}}, {4{sprL[3]}}, {4{sprL[4]}}, {4{sprL[5]}}, {4{sprL[6]}}, {4{sprL[7]}}, {4{sprL[8]}} };
                                    end
                                end
                                else if (extendedOpL[m] == 467) // mtspr
                                begin
`ifdef LD_CHANGE
    $display("L MTSPR");
`endif
                                    if ((sprL[m][0:4] == 5'b00001 || sprL[m][0:4] == 5'b01000 || sprL[m][0:4] == 5'b01001) && sprL[m][5:9] == 0)
                                    begin
                                        iid[m] = `MTSPR;

                                        r0[m] = regZero[rDL[i]] ? {1'b0, rDL[m]} : `ZERO_LOAD;
                                        r1[m] = `ZERO_LOAD;

                                        idata[m] = sprL[m][0:4];
                                    end
                                 end
                                 else if (extendedOpL[m] == 339) // mfspr
                                 begin
`ifdef LD_CHANGE
    $display("L MFSPR");
`endif
                                     if ((sprL[m][0:4] == 5'b00001 || sprL[m][0:4] == 5'b01000 || sprL[m][0:4] == 5'b01001) && sprL[m][5:9] == 0)
                                     begin
                                         iid[m] = `MFSPR;

                                         w0[m] = rDL[m];
                                         w1[m] = `ZERO_LOAD;

                                         idata[m] = sprL[m][0:4];
                                     end
                                end
                            end
                            14: // addi
                            begin
`ifdef LD_CHANGE
    $display("L ADDI %d, %d, %d", rDL, rAL, { {48{simmL[3]}}, simmL });
`endif
                                iid[m] = `ADDI;
            
                                m0[m] = `ZERO_LOAD;
            
                                if (rAL == 0)
                                    r0[m] = `ZERO_LOAD;
                                else
                                    r0[m] = regZero[rAL] ? {1'b0, rAL} : `ZERO_LOAD;
            
                                r1[m] = `ZERO_LOAD;
            
                                w0[m] = {1'b0, rDL};
                                w1[m] = `ZERO_LOAD;
            
                                // store OE in sub id
                                idata[m] = { {48{simmL[3]}}, simmL };
                            end
                            18: // bx (this is the only instruction that is evaluated immediately)
                            begin
`ifdef LD_CHANGE
    $display("L BX");
`endif
                                iid[m] = `BX;

                                if (bAAL)
                                    idata[m] = { {48{bLIL[2]}}, bLIL, 2'b00 };
                                else
                                    idata[m] = ipc[m] + { {48{bLIL[2]}}, bLIL, 2'b00 };
            
                                // store bLK, rD, and rA in sub id (only bLK is needed)
                                siid[m] = 0 | bLKL;

                                // evaluate the branch
                                bp[i] = 1;
                            end
                            16: // bc
                            begin
`ifdef LD_CHANGE
    $display("L BC");
`endif
                                iid[m] = `BC;
            
                                // store bLK, rD, and rA in sub id (only bLK is needed)
                                siid[m] = 0 | rAL << 6 | rDL << 1 | bLKL;
            
                                // store the immediate
                                if (bAAL)
                                    idata[m] = { {48{bDL[2]}}, bDL, 2'b00 };
                                else
                                    idata[m] = ipc[m] + { {48{bDL[2]}}, bDL, 2'b00 };

                                // evaluate the branch
                                bp[i] = 1;
                            end
                            19: // bclr
                            begin
`ifdef LD_CHANGE
    $display("L BCLR");
`endif
                                if (extendedOpL == 16)
                                begin
                                    iid[m] = `BCLR;
            
                                    // store bLK, rD, and rA in sub id
                                    siid[m] = 0 | rAL << 6 | rDL << 1 | bLKL;

                                    // evaluate the branch
                                    bp[i] = 1;
                                end
                            end
                            58: // ld
                            begin
                                case (loadExOpL)
                                    2'b00: // ld
                                    begin
`ifdef LD_CHANGE
    $display("L LD %d, %d", rDL, rAL, $signed({ {48{loadDSL[3]}}, loadDSL, 2'b0 }));
`endif
                                        iid[m] = `LD;
            
                                        m0[m] = `LD_LOAD;
            
                                        r0[m] = (rAL != 0 && regZero[rAL]) ? {1'b0, rAL} : `ZERO_LOAD;
                                        r1[m] = `ZERO_LOAD;
            
                                        w0[m] = {1'b0, rDL};
                                        w1[m] = `ZERO_LOAD;
            
                                        idata[m] = { {48{loadDSL[3]}}, loadDSL, 2'b0 };
                                    end
                                    2'b01: // ldu
                                    begin
`ifdef LD_CHANGE
    $display("L LDU");
`endif
                                        if (rAL != 0 && rAL != rDL) // check the instruction is valid
                                        begin
                                            iid[m] = `LDU;
            
                                            m0[m] = `LD_LOAD;
            
                                            r0[m] = regZero[rAL] ? {1'b0, rAL} : `ZERO_LOAD;
                                            r1[m] = `ZERO_LOAD;
            
                                            w0[m] = {1'b0, rDL};
                                            w1[m] = {1'b0, rAL};
            
                                            idata[m] = { {48{loadDSL[3]}}, loadDSL, 2'b0 };
                                        end
                                    end
                                   
                                endcase
                            end
                            467: // mtspr
                            begin
                                if (sprL[5:9] == 0)
                                begin
                                    case (extendedOpL)
                                        467: // mtspr
                                        begin
`ifdef LD_CHANGE
    $display("L MTSPR");
`endif
                                            if (sprL[0:4] == 5'b00001 || sprL[0:4] == 5'b01000 || sprL[0:4] == 5'b01001)
                                            begin
                                                iid[m] = `MTSPR;
            
                                                r0[m] = regZero[rDL] ? {1'b0, rDL} : `ZERO_LOAD;
                                                r1[m] = `ZERO_LOAD;
            
                                                idata[m] = sprL[0:4];
                                            end
                                        end
                                        339:
                                        begin // mfspr
`ifdef LD_CHANGE
    $display("L MFSPR");
`endif
                                            if (sprL[0:4] == 5'b00001 || sprL[0:4] == 5'b01000 || sprL[0:4] == 5'b01001)
                                            begin
                                                iid[m] = `MFSPR;
            
                                                w0[m] = rDL;
                                                w1[m] = `ZERO_LOAD;
            
                                                idata[m] = sprL[0:4];
                                            end
                                        end
                                    endcase
                                end
                            end
                            62: // std
                            begin
`ifdef LD_CHANGE
    $display("L STD");
`endif
                                if (loadExOpL == 0)
                                begin
                                    iid[m] = `STD;
            
                                    r0[m] = regZero[rDL] ? {1'b0, rDL} : `ZERO_LOAD;
                                    r1[m] = (rAL != 0 && regZero[rAL]) ? {1'b0, rAL} : `ZERO_LOAD;
            
                                    idata[m] = { {48{loadDSL[3]}}, loadDSL, 2'b0 };
                                end
                            end
                            17: // sc
                            begin
                                if (bAAL && LEVL[0:5] == 0)
                                begin
`ifdef LD_CHANGE
    $display("L SC");
`endif
                                    iid[m] = `SC;
            
                                    m0[m] = `ZERO_LOAD;
            
                                    r0[m] = regZero[0] ? {1'b0, 4'b0000} : `ZERO_LOAD;
                                    r1[m] = regZero[3] ? {1'b0, 4'b0011} : `ZERO_LOAD;
            
                                    w0[m] = `ZERO_LOAD;
                                    w1[m] = `ZERO_LOAD;
                                end
                            end
                        endcase
            
                       // update zero load
                       if (w0[m] != `ZERO_LOAD)
                           regZero[w0[m][1:5]] = 1'b1;
            
                       if (w1[m] != `ZERO_LOAD)
                           regZero[w1[m][1:5]] = 1'b1;
                    end

                    // check if loaded / loading data is modified
                    if (m0[m] != `ZERO_MEM_LOAD && m0[m] == wm0[i][2:62])
                    begin
                        if (mA[m][0:1] == 2'b10)
                            lock = 1;

                        mA[m] = {1'b0, 1'b1, inA[i][2:65]};
                    end

                    /**************
                    BRANCH PREDICTION
                    (same as before)
                    **************/

                    if (bp[m] == 1)
                    begin
                        // hash the number (Knuth Algorithm)
                        hash = (ipc[m] * 2654435761 >> 32) % `PREDICTION_TABLE_H;

                        // starting value for sum
                        hashSum = predictionTable[(iid[m] - `BX) * `PREDICTION_TABLE_W * `PREDICTION_TABLE_H + hash * `PREDICTION_TABLE_W];

                        // sum up the weights
                        for (k = 1; k < `PREDICTION_TABLE_W; k=k+1)
                            hashSum = hashSum + predictionTable[(iid[m] - `BX) * `PREDICTION_TABLE_W * `PREDICTION_TABLE_H + hash * `PREDICTION_TABLE_W + k] * (predictionHistory[(iid[m] - `BX)][k] ? 1 : -1);

                        if (hashSum < $signed(0))
                            bp[m] = 0; // not taken
                        else
                        begin
                            // check the type of branch to take
                            case (iid[m])
                                `BX:
                                 begin
                                     bp[m] = 1;

                                     bp[m] = idata[m]; // store the branch location

                                     pc = idata[m];
                                 end
                                `BC:
                                 begin
                                     bp[m] = 1;

                                     bp[m] = idata[m]; // store the branch location

                                     pc = idata[m];
                                 end
                                `BCLR:
                                 begin
                                     bp[m] = 1;

                                     bp[m] = link; // store the branch location

                                     pc = link;
                                 end
                            endcase

                            bp[m][0] = 1; // taken

                            branch = 1;

                            lock2 = 1;
                        end
                    end
                end

               // destroy instructions based on the branch
               for (m = m; $signed(0) <= m && lock2 == 1; m = m - 1)
               begin
                   iid[m] = `INVALID;

                   m0[m] = `ZERO_MEM_LOAD;

                   r0[m] = `ZERO_LOAD;
                   r1[m] = `ZERO_LOAD;
    
                   w0[m] = `ZERO_LOAD;
                   w1[m] = `ZERO_LOAD;

                   wm0[m] = `ZERO_MEM_LOAD;

                   ipc[m] = 0;

                   bp[m] = 0;
                end

                // if the lock is set, destroy all load dependencies and reset the loop
                if (lock == 1)
                begin
                    i = queueSize - 1; // reset the loop

                    for (m = 0; m < queueSize; m=m+1)
                    begin
                        if (inA[m][0] == 1'b1 && inA[m][1] == 1'b1)
                            inA[m] = 0;

                        if (inB[m][0] == 1'b1 && inB[m][1] == 1'b1)
                            inB[m] = 0;

                        if (mA[m][0] == 1'b1 && mA[m][1] == 1'b1)
                            mA[m] = 0;
                    end
                end
            end
        end

        /**************
        EXECUTE ORDER 66
               _.-'~~~~~~`-._
              /      ||      \
             /       ||       \
            |        ||        |
            | _______||_______ |
            |/ ----- \/ ----- \|
           /  (     )  (     )  \
          / \  ----- () -----  / \
         /   \      /||\      /   \
        /     \    /||||\    /     \
       /       \  /||||||\  /       \
      /_        \o========o/        _\
        `--...__|`-._  _.-'|__...--'
                |    `'    |
        **************/

        // find the number of instructions able to execute
        numWrite = 0;
        numMemWrite = 0;

        lock = 0;

        for (i = queueSize - 1; $signed(0) <= i && lock == 0; i=i-1)
        begin
            // check all expected values will have something in them upon execution
            
            // memory check
            if (m0[i] != `ZERO_MEM_LOAD && ((mA[i][0] == 1'b1 && mA[i][1] == 1'b0) || mA[i] == 0))
                lock = 1;

            // register A check
            if (r0[i] != `ZERO_LOAD && ((inA[i][0] == 1'b1 && inA[i][1] == 1'b0) || inA[i] == 0))
                lock = 1;

            // register B check
            if (r1[i] != `ZERO_LOAD && ((inB[i][0] == 1'b1 && inB[i][1] == 1'b0) || inB[i] == 0))
                lock = 1;

            // STD check
            if (iid[i] == `STD  && !(wm0[i][0] == 1'b0 && wm0[i][1] == 1'b1))
                lock = 1;

            // write conflict check
            
            lock2 = 0;

            // write 0 check
            if (w0[i] != `ZERO_LOAD && lock == 0)
            begin
                for (m = 0; m < numWrite && lock2 == 0; m=m+1)
                begin
                    if (writeReg[m] == w0[i])
                    begin
                        lock2 = 1;
                    end
                end

                if (lock2 == 0)
                begin
                    numWrite = numWrite + 1;

                    // assign a write port
                    if (numWrite <= `MAX_WRITE)
                        writeReg[numWrite - 1] = w0[i];
                end
            end

            lock2 = 0;

            // write 1 check
            if (w1[i] != `ZERO_LOAD && lock == 0)
            begin
                for (m = 0; m < numWrite && w1[i] != `ZERO_LOAD && lock2 == 0; m=m+1)
                begin
                    if (writeReg[m] == w1[i])
                    begin
                        lock2 = 1;
                    end
                end

                if (lock2 == 0)
                begin
                    numWrite = numWrite + 1;

                    // assign a write port
                    if (numWrite <= `MAX_WRITE)
                        writeReg[numWrite - 1] = w1[i];
                end
            end

            // write mem conflict

            lock2 = 0;

            if (wm0[i] != `ZERO_MEM_LOAD && lock == 0)
            begin
                for (m = 0; m < numMemWrite && lock2 == 0; m=m+1)
                begin
                    if (writeMem[m] == wm0[i])
                    begin
                        lock2 = 1;
                    end
                end

                if (lock2 == 0)
                begin
                    numMemWrite = numMemWrite + 1;

                    // assign a write port
                    if (numMemWrite <= `MAX_MEM_WRITE)
                        writeMem[numMemWrite - 1] = wm0[i][2:62];
                end
            end

            // check the number of writes and mem writes is feasible
            if (`MAX_WRITE < numWrite || `MAX_MEM_WRITE < numMemWrite)
                lock = 1;
        end

        if (lock == 0)
            executeNum = $signed(queueSize);
        else
            executeNum = $signed(queueSize) - i - 2;

        /**************
        Run the commands
    .--._                      __.=--,-----.______
   /     `-._             _,--',-' |/  ___.-------`--._________       __
  / ____ (\) `-.      _.-'_    |   |  /   |          | H H H H \-----'  `---...
 / /\ \ `---.__/     /__/  \_______|-'___ |_         |_H_H_H_H_/-----.__,---'''
 \/`.\ \               /  _/| __  |U   |____|--------'
   _ `\ \            _/,-'  |'__`.|           | |
  /_\.-,.\          /(O    ( (__) ).        ,'  |
 |   |(())_        /,-'     `.__,'  `.    ,'  ,'
 |___| \'  \      //          `.    ,.`.,'  ,'
 | | | /`.  \    //             `. (  /'`.,'
 | | |/   `. \,.//                `.':  .':.     _
 | |.(   /_ >(())                   `.`.|  `:.  | |.
 | || |_/[_]| `'                     |`.`. /  `.|  |
 |_|| |_,| | \                       |  `.`    /   |.
  `.|   ||#|  )                      |o __`._-'--._ |
       _|`-' (           ,.          `-(   | ``:.. \|
       | `-._ )     ___,(())          -=\  |---::|_/|.
       |  __ (_,---'   _,:' \            \_|_,-'''   |
       |_/  \'..::''_,'  (   \           |#          |
        \\__/.__,--'      `.  \          |#          |
                            `. \         |#          |
                             `. \        |#          |
                              `. \ __,-. |#          |
                                )   _,/  |#         |'
                               (_,-'-----+====o--- _|
                                         |#       [_
                                         |         |'
                                         |         |
                                         |_       |'
                                           |     /
                                           |    /
                                           |   /
                                           |  /
                                           |,'
        **************/

        // lock called if a branch is found during execution
        lock2 = 0;        

        for (i = queueSize - 1; $signed(queueSize - executeNum) <= i && lock2 == 0; i=i-1)
        begin
            // find the ports to write to

            lock = 0;

            // w0
            if (w0[i] != `ZERO_LOAD)
            begin
               for (m = 0; m < $signed((`MAX_WRITE < numWrite) ? `MAX_WRITE : numWrite) && lock == 0; m=m+1)
                begin
                    if (w0[i][1:5] == writeReg[m])
                    begin
                        lock = 1;

                        w0[i] = m;
                    end
                end
            end

            lock = 0;

            // w1
            if (w1[i] != `ZERO_LOAD)
            begin
                for (m = 0; m < $signed((`MAX_WRITE < numWrite) ? `MAX_WRITE : numWrite) && lock == 0; m=m+1)
                begin
                    if (w1[i][1:5] == writeReg[m])
                    begin
                        lock = 1;

                        w1[i] = m;
                    end
                end
            end

            lock = 0;

            // wm0
            if (wm0[i] != `ZERO_MEM_LOAD)
            begin
               for (m = 0; m < $signed((`MAX_MEM_WRITE < numMemWrite) ? `MAX_MEM_WRITE : numMemWrite) && lock == 0; m=m+1)
                begin
                    if (wm0[i][2:62] == writeMem[m])
                    begin
                        lock = 1;

                        wm0[i] = m;
                    end
                end
            end

            case (iid[i])
                `ADD:
                begin
`ifdef EXEC
    $display("O ADD");
`endif

                    writeData[w0[i]] = inA[i][2:65] + inB[i][2:65];
                    portUsed[w0[i]] = 1;

                    if (siid[i][10])
                    begin
                         xer[33] = ((inA[i][2:65] + inB[i][2:65]) & 1 << 63) != 0 ^ (({inA[i][2], inA[i][2:65]} + {inB[i][2], inB[i][2:65]}) & 1 << 64) != 0;
                         xer[32] = xer[32] | xer[33];
                    end

                    if (siid[i][9])
                    begin
                        condition[0] = $signed(writeData[w0[i]]) < 0;
                        condition[1] = $signed(writeData[w0[i]]) > 0;
                        condition[2] = writeData[w0[i]] == 0;
                        condition[3] = xer[32];
                    end
                end
                `ORX:
                begin
`ifdef EXEC
    $display("O ORX");
`endif
                    writeData[w0[i]] = inA[i][2:65] | inB[i][2:65];
                    portUsed[w0[i]] = 1;

                    if (siid[i][10])
                    begin
                        condition[0] = $signed(writeData[w0[i]]) < 0;
                        condition[1] = $signed(writeData[w0[i]]) > 0;
                        condition[2] = writeData[w0[i]] == 0;
                        condition[3] = xer[32];
                    end
                end
                `ADDI:
                begin
`ifdef EXEC
    $display("O ADDI %d", idata[i]);
`endif
                    writeData[w0[i]] = inA[i][2:65] + idata[i];
                    portUsed[w0[i]] = 1;
                end
                `BX:
                begin
`ifdef EXEC
    $display("O BX %d %d, %d", idata[i], siid[i][10], ipc[i]);
`endif
                    // check the branch prediction was correct
                    bp[i][1] = (bp[i][0] && bp[i][1:64] == idata[i]);
                    bp[i][0] = 0;

                    if (siid[i][10])
                        link = ipc[i] + 4;

                    if (!bp[i][1]) // branch prediction was wrong
                    begin
                        // branch detected
                        branch = 1;
                        lock2 = 1;

                        // flush the queue
                        executeNum = queueSize;

                        pc = idata[i];
                    end

                    // branch taken
                    bp[i][0] = 1;
                end
                `BC:
                begin
`ifdef EXEC
    $display("O BC");
`endif
                    // check the branch prediction was correct
                    bp[i][1] = (bp[i][0] && bp[i][1:64] == idata[i]);
                    bp[i][0] = 0;

                    if (!siid[i][7])
                        counter = counter - 1;

                    if ((siid[i][7] || (counter != 0 ^ siid[i][8])) && (siid[i][5] || (condition[siid[i][0:4]] == siid[i][6])))
                    begin
                        if (!bp[i][1]) // branch prediction was wrong
                        begin
                            // branch detected
                            branch = 1;
                            lock2 = 1;

                            // flush the queue
                            executeNum = queueSize;

                            pc = idata[i];
                        end

                        // branch taken
                        bp[i][0] = 1;
                    end
                    else if (bp[i][1]) // branch prediction was wrong
                    begin
                        // branch detected
                        branch = 1;
                        lock2 = 1;

                        // flush the queue
                        executeNum = queueSize;

                        // restore program to its correct state
                        pc = ipc[i] + 4;
                    end

                    if (siid[i][10])
                        link = ipc[i] + 4;
                end
                `BCLR:
                begin
`ifdef EXEC
    $display("O BCLR");
`endif
                    // check the branch prediction was correct
                    bp[i][1] = (bp[i][0] && bp[i][1:64] == link);
                    bp[i][0] = 0;

                    if(!siid[i][7])
                        counter = counter - 1;

                    if ((siid[i][7] || (counter != 0 ^ siid[i][8])) && (siid[i][5] || (condition[siid[i][0:4]] == siid[i][6])))
                    begin
                        if (!bp[i][1]) // branch prediction was wrong
                        begin
                            // branch detected
                            branch = 1;
                            lock2 = 1;

                            // flush the queue
                            executeNum = queueSize;

                            pc = link;
                        end

                        // branch taken
                        bp[i][0] = 1;
                    end
                    else if (bp[i][1]) // branch prediction was wrong
                    begin
                        // branch detected
                        branch = 1;
                        lock2 = 1;

                        // flush the queue
                        executeNum = queueSize;

                        // restore program to its correct state
                        pc = ipc[i] + 4;
                    end

                    if (siid[i][10])
                        link = ipc[i] + 4;
                end
                `LD:
                begin
`ifdef EXEC
    $display("O LD %d", m0[i] << 3);
`endif
                    writeData[w0[i]] = mA[i][2:65];
                    portUsed[w0[i]] = 1;
                end
                `LDU:
                begin
`ifdef EXEC
    $display("O LDU");
`endif
                    writeData[w0[i]] = mA[i][2:65];
                    writeData[w1[i]] = idata[i];

                    portUsed[w0[i]] = 1;
                    portUsed[w1[i]] = 1;
                end
                `MTSPR:
                begin
`ifdef EXEC
    $display("O MTSPR");
`endif
                    case (idata[i])
                        5'b00001:
                        begin
                            xer = inA[i][2:65];
                        end
                        5'b01000:
                        begin
                            link = inA[i][2:65];
                        end
                        5'b01001:
                        begin
                            counter = inA[i][2:65];
                        end
                    endcase
                end
                `MFSPR:
                begin
`ifdef EXEC
    $display("O MFSPR");
`endif
                    portUsed[w0[i]] = 1;

                    case (idata[i])
                        5'b00001:
                        begin
                            writeData[w0[i]] = xer;
                        end
                        5'b01000:
                        begin
                            writeData[w0[i]] = link;
                        end
                        5'b01001:
                        begin
                            writeData[w0[i]] = counter;
                        end
                    endcase
                end
                `MTCRF:
                begin
`ifdef EXEC
    $display("O MTCRF");
`endif
                    condition = (inA[i][34:65] & idata[i]) | (counter & !idata[i]);
                end
                `STD:
                begin
`ifdef EXEC
    $display("O STD");
`endif
                    portMemUsed[wm0[i]] = 1;

                    writeMemData[wm0[i]] = inA[i];
                end
                `SC:
                begin
`ifdef EXEC
    $display("O SC");
`endif
                    case(inA[i][2:65])
                        0:
                            $display("%c", inB[i][58:65]);
                        1:
                        begin
                            haltR = 1;

                            // branch detected (not really but same behavior)
                            branch = 1;
                            lock2 = 1;
`ifdef WRITE_TABLE
    // output the table file
    $writememh("table.txt", predictionTable);

    // output the history file
    $writememh("history.txt", predictionHistory);
`endif

                        end
                        2:
                            $display("%h", inB[i][2:65]);
                    endcase
                end
            endcase

`ifdef EXEC
    $display("PC %d", ipc[i]);
`endif

            // update all those waiting on this data

            // update for w0
            if (w0[i] != `ZERO_LOAD)
            begin
                lock = 0;

                for (m = i - 1; $signed(0) <= m && lock == 0; m=m-1)
                begin
                    // look to see if this element in the queue is waiting on this instruction

                    // inA
                    if (r0[m] == writeReg[w0[i]] && inA[m][0] == 1'b1 && inA[m][1] == 1'b1)
                        inA[m] = {1'b0, 1'b1, writeData[w0[i]]};

                    // inB
                    if (r1[m] == writeReg[w0[i]] && inB[m][0] == 1'b1 && inB[m][1] == 1'b1)
                        inB[m] = {1'b0, 1'b1, writeData[w0[i]]};

                    // check that w0 is not written to by this instruction
                    if (w0[m] == writeReg[w0[i]] || w1[m] == writeReg[w0[i]])
                        lock = 1;
                end
            end

            // update for w1
            if (w1[i] != `ZERO_LOAD)
            begin
                lock = 0;

                for (m = i - 1; $signed(0) <= m && lock == 0; m=m-1)
                begin
                    // look to see if this element in the queue is waiting on this instruction

                    // inA
                    if (r0[m] == writeReg[w1[i]] && inA[m][0] == 1'b1 && inA[m][0] == 1'b1)
                        inA[m] = {1'b0, 1'b1, writeData[w1[i]]};

                    // inB
                    if (r1[m] == writeReg[w1[i]] && inB[m][0] == 1'b1 && inB[m][1] == 1'b1)
                        inB[m] = {1'b0, 1'b1, writeData[w1[i]]};

                    // check that w0 is not written to by this instruction
                    if (w0[m] == writeReg[w1[i]] || w1[m] == writeReg[w1[i]])
                        lock = 1;
                end
            end

            // update prediction table
            if (bp[i][1] != bp[i][0]) // prediction was incorrect or correlation is low
            begin
                hash = (ipc[i] * 2654435761 >> 32) % `PREDICTION_TABLE_H;

                predictionTable[(iid[i] - `BX) * `PREDICTION_TABLE_W * `PREDICTION_TABLE_H + hash * `PREDICTION_TABLE_W] = predictionTable[(iid[i] - `BX) * `PREDICTION_TABLE_W * `PREDICTION_TABLE_H + hash * `PREDICTION_TABLE_W] + $signed(bp[i][0] ? 1 : -1);

                for (m = 1; m < `PREDICTION_TABLE_W; m=m+1)
                begin
                    predictionTable[(iid[i] - `BX) * `PREDICTION_TABLE_W * `PREDICTION_TABLE_H + hash * `PREDICTION_TABLE_W + m] = predictionTable[(iid[i] - `BX) * `PREDICTION_TABLE_W * `PREDICTION_TABLE_H + hash * `PREDICTION_TABLE_W + m] + $signed(bp[i][0] == predictionHistory[(iid[i] - `BX)][m] ? 1 : -1);
                end

                predictionHistory[(iid[i] - `BX)] = predictionHistory[(iid[i] - `BX)] << 1 | bp[i][0];
            end
        end

        // check all ports were used and, if not, fix the data
        for (i = 0; i < numWrite; i=i+1)
        begin
            // shift everything to the left if the port is not used
            if (!portUsed[i])
            begin
                for (m = i + 1; m < numWrite; m=m+1)
                begin
                   portUsed[m - 1] = portUsed[m];
                   writeReg[m - 1] = writeReg[m];
                   writeData[m - 1] = writeData[m];
                end

                i = i - 1;
                numWrite = numWrite - 1;
            end
        end

        // check all memory write ports were used and, if not, fix the data
        for (i = 0; i < numMemWrite; i=i+1)
        begin
            // shift everything to the left if the port is not used
            if (!portMemUsed[i])
            begin
                for (m = i + 1; m < numWrite; m=m+1)
                begin
                   portMemUsed[m - 1] = portMemUsed[m];
                   writeMem[m - 1] = writeMem[m];
                   writeMemData[m - 1] = writeMemData[m];
                end

                i = i - 1;
                numMemWrite = numMemWrite - 1;
            end
        end

        // restore the old data where needed
        for (i = numWrite; i < `MAX_WRITE; i=i+1)
        begin
            writeReg[i] = lastWriteReg[i];  
            writeData[i] = lastWriteData[i];    
        end

        // remove data from queue
        queueSize = queueSize - executeNum;

        // check for anything writing the value zero
        for (i = 0; i < numWrite; i=i+1)
        begin
            lock = 0;

            if (writeData[i] == 0)
            begin
                // check for if an instruction writes to this data
                for (m = 0;m < queueSize && lock == 0; m = m + 1)
                begin
                    if (w0[m] == writeReg[i] || w1[m] == writeReg[i])
                        lock = 1;
                end
            end
            else
                lock = 1;

            if (lock == 1)
                regZero[writeReg[i]] = 1;
        end

        // uncheck port swap
        portSwap <= 0;

        // check if writing makes two ports the same (only happens if one write port is being used)
        if (numWrite == 1 && writeReg[0] == lastWriteReg[1])
        begin
            portSwap <= 1;

            // move data over to 1
            writeReg[1] = writeReg[0];
            writeData[1] = writeData[0];

            // preserve the old data
            writeReg[0] = lastWriteReg[0];
            writeData[0] = lastWriteData[0];
        end

        if (oddCycle != 0 && !(oddCycle == 1 && expectedVal == 0))
            oddCycle = oddCycle - 1;

        if (!branch)
        begin
            expectedVal = pcOffset / 4; // update the expected value for the next cycle
        end
        else
        begin
            expectedVal = 0; // nothing expected next cycle
            oddCycle = pc % 8 == 0 ? 0 : 2;
        end

        // update if no branch
        if (!branch)
            pc = pc + pcOffset;
        else if (oddCycle == 2) // round pc down if it's in an odd cycle
            pc = pc - 4;

        // copy over the data needed for reading
        for (i = 0; i < `MAX_MEM_READ; i = i + 1)
        begin
            lastMemRead[i] <= readMem[i];
        end

        for (i = 0; i < `MAX_READ; i = i + 1)
        begin
            lastReadReg[i] <= readReg[i];
        end

        for (i = 0; i < `MAX_WRITE; i = i + 1)
        begin
            lastWriteReg[i] <= writeReg[i];
            lastWriteData[i] <= writeData[i];
        end

        for (i = 0; i < `MAX_MEM_WRITE; i = i + 1)
        begin
            lastMemWrite[i] <= writeMem[i];
            lastMemWriteData[i] <= writeMemData[i];

            historyWriteEn <= memWriteEn;
            historyWriteMem <= lastMemWrite[i];
            historyWriteData <= lastMemWriteData[i];
        end

        oldNumMemWrite <= numMemWrite;

        oldNumWrite <= numWrite;

        lastPc <= pc;
    end
endmodule

