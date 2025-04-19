module top_rv32i_soc
  (
    // Power pads
  //  input  VDD1,     // external VDD pad 1
  //  input  VDD2,     // external VDD pad 2
  //  input  VSS1,     // external VSS pad 1
  //  input  VSS2,     // external VSS pad 2

    // Clock pad
    input  CLK_PAD,  // external clock pad

    // Reset and scan signals
    input  RESET_N_PAD,          // external reset (active low)
    //input  [5:0] SCAN_IN_PAD,    // external scan_in pins
    //output [5:0] SCAN_OUT_PAD,   // external scan_out pins
    //input  SCAN_ENABLE_PAD,      // external scan_enable
    //input  MODE_PAD,             // external mode signal

    // SPI pad signals
    output O_FLASH_SCLK_PAD,     // external SPI flash serial clock
    output O_FLASH_CS_N_PAD,     // external SPI flash chip‐select (active low)
    output O_FLASH_MOSI_PAD,     // external SPI flash MOSI
    input  I_FLASH_MISO_PAD,     // external SPI flash MISO

    // UART pad signals
    output O_UART_TX_PAD,        // external UART TX
    input  I_UART_RX_PAD,        // external UART RX

    // 24‐bit GPIO pads (bidirectional)
    inout  [23:0] IO_DATA_PAD,  // external GPIO pads

    output O_PWM_PAD
  );

  //-------------------------------------------------------------------------
  // Internal power nets (choose one VDD and one VSS to supply the core)
  //-------------------------------------------------------------------------

  
  // For illustration we simply tie the internal supply nets to one of the pads.
  // (In a full-chip design, the power grid may be more complex.)

  //-------------------------------------------------------------------------
  // Instantiate power pad cells
  //-------------------------------------------------------------------------
  //PVDD2DGZ u_vdd1 (.VDDPST(VDD1));
  //PVDD2DGZ u_vdd2 (.VDDPST(VDD2));
  //PVSS2DGZ u_vss1 (.VSSPST(VSS1));
  //PVSS2DGZ u_vss2 (.VSSPST(VSS2));

  //-------------------------------------------------------------------------
  // Instantiate clock pad cell
  //-------------------------------------------------------------------------
  wire clk_internal;
  wire clk_dummy; // not used further
  PDXO03DG u_clk_pad (
      .XIN  (CLK_PAD),
      .XC   (clk_internal)
  );

  //-------------------------------------------------------------------------
  // Attach Reset and Scan signals using I/O pad cells (PDD24DGZ)
  //-------------------------------------------------------------------------
  // For input pads, we set OEN = 1 and tie I to a constant.
  // The buffered output (C) is then used internally.

  wire reset_n_internal;
  PDD24DGZ u_reset_pad (
      .I   (1'b0),
      .OEN (1'b1),
      .PAD (RESET_N_PAD),
      .C   (reset_n_internal)
  );

  // Scan_in (6-bit input)
  //wire [5:0] scan_in_internal;
  //genvar j;
  //generate
  //  for (j = 0; j < 6; j = j + 1) begin : scan_in_pad_gen
  //    PDD24DGZ u_scan_in_pad (
  //        .OEN (1'b1),
  //       .PAD (SCAN_IN_PAD[j]),
  //       .C   (scan_in_internal[j])
  //    );
  //  end
 // endgenerate

  // Scan_out (6-bit output)
// wire [5:0] scan_out_internal;
//  generate
//    for (j = 0; j < 6; j = j + 1) begin : scan_out_pad_gen
//      PDD24DGZ u_scan_out_pad (
//          .I   (scan_out_internal[j]),
//          .OEN (1'b0),    // drive output
//          .PAD (SCAN_OUT_PAD[j])      // output buffer not used internally
//      );
//    end
// endgenerate

  // Scan_enable input
//  wire scan_enable_internal;
//  PDD24DGZ u_scan_enable_pad (
//      .I   (1'b0),
//      .OEN (1'b1),
//      .PAD (SCAN_ENABLE_PAD),
//      .C   (scan_enable_internal)
//  );

  // Mode input
//  wire mode_internal;
//  PDD24DGZ u_mode_pad (
//      .I   (1'b0),
//     .OEN (1'b1),
//      .PAD (MODE_PAD),
//      .C   (mode_internal)
//  );

  //-------------------------------------------------------------------------
  // Attach SPI signals using I/O pad cells
  //-------------------------------------------------------------------------
  // o_flash_sclk (output)
  wire o_flash_sclk_internal;
  PDD24DGZ u_flash_sclk_pad (
      .I   (o_flash_sclk_internal),
      .OEN (1'b0),
      .PAD (O_FLASH_SCLK_PAD),
      .C   ()
  );

  // o_flash_cs_n (output)
  wire o_flash_cs_n_internal;
  PDD24DGZ u_flash_cs_n_pad (
      .I   (o_flash_cs_n_internal),
      .OEN (1'b0),
      .PAD (O_FLASH_CS_N_PAD),
      .C   ()
  );

  // o_flash_mosi (output)
  wire o_flash_mosi_internal;
  PDD24DGZ u_flash_mosi_pad (
      .I   (o_flash_mosi_internal),
      .OEN (1'b0),
      .PAD (O_FLASH_MOSI_PAD),
      .C   ()
  );

  // i_flash_miso (input)
  wire i_flash_miso_internal;
  PDD24DGZ u_flash_miso_pad (
      .I   (1'b0),
      .OEN (1'b1),
      .PAD (I_FLASH_MISO_PAD),
      .C   (i_flash_miso_internal)
  );

  //-------------------------------------------------------------------------
  // Attach UART signals using I/O pad cells
  //-------------------------------------------------------------------------
  // o_uart_tx (output)
  wire o_uart_tx_internal;
  PDD24DGZ u_uart_tx_pad (
      .I   (o_uart_tx_internal),
      .OEN (1'b0),
      .PAD (O_UART_TX_PAD),
      .C   ()
  );

  // i_uart_rx (input)
  wire i_uart_rx_internal;
  PDD24DGZ u_uart_rx_pad (
      .I   (1'b0),
      .OEN (1'b1),
      .PAD (I_UART_RX_PAD),
      .C   (i_uart_rx_internal)
  );

  //-------------------------------------------------------------------------
  // Attach the 24-bit GPIO signals
  //-------------------------------------------------------------------------
  // For a true bidirectional pad you would normally control the pad’s OEN 
  // with a direction signal from the core. In this simple example we assume
  // the GPIO are always driven (OEN = 0) when used as outputs.
  // (If you need bidirectionality, modify to drive OEN from an internal "oe".)
  wire [23:0] i_gpio_internal;
  wire [23:0] o_gpio_internal;
  wire [23:0] en_gpio_internal;

  // The internal net "gpio_internal" will connect to the chip’s io_data port.
  // Here we instantiate one pad per bit.
  genvar k;
  generate
    for (k = 0; k < 24; k = k + 1) begin : gpio_pad_gen
      PDD24DGZ u_gpio_pad (
          .I   (o_gpio_internal[k]),
          .OEN (~en_gpio_internal[k]),      // for example, force output (modify as needed)
          .PAD (IO_DATA_PAD[k]),
          .C   (i_gpio_internal[k])           // not used internally in this example
      );
    end
  endgenerate


  //-------------------------------------------------------------------------
  // PWN signals 
  //-------------------------------------------------------------------------
  // pwm_pad_o
  wire pwm_pad_o_internal;
  PDD24DGZ u_pwm_pad (
      .I   (pwm_pad_o_internal),
      .OEN (1'b0),
      .PAD (O_PWM_PAD),
      .C   ()
  );


  //-------------------------------------------------------------------------
  // Instantiate the bare RISC-V chip
  //-------------------------------------------------------------------------
  // The internal nets from the pads are connected to the chip instance.
  rv32i_soc u_rv32i_soc (
      .clk         (clk_internal),
      .reset_n     (reset_n_internal),
   //   .scan_in     (scan_in_internal),
   //   .scan_out    (scan_out_internal),
   //   .scan_enable (scan_enable_internal),
   //   .mode        (mode_internal),

      // SPI signals
      .o_flash_sclk (o_flash_sclk_internal),
      .o_flash_cs_n (o_flash_cs_n_internal),
      .o_flash_mosi (o_flash_mosi_internal),
      .i_flash_miso (i_flash_miso_internal),

      // UART signals
      .o_uart_tx    (o_uart_tx_internal),
      .i_uart_rx    (i_uart_rx_internal),

      // 24-bit GPIO signals 
      .i_gpio(i_gpio_internal),
      .o_gpio(o_gpio_internal),
      .en_gpio(en_gpio_internal),

      .pwm_pad_o(pwm_pad_o_internal)
  );

  //PCORNER u_corner1 (
  //);

  //PCORNER u_corner2 (
  //);

  //PCORNER u_corner3 (
  //);

  //PCORNER u_corner4 (
  //);

endmodule
