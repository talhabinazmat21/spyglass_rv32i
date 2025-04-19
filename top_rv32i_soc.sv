module top_rv32i_soc
  (


    // Clock pad
    input  CLK_PAD,  // external clock pad

    // Reset and scan signals
    input  RESET_N_PAD,          // external reset (active low)
    //input  [5:0] SCAN_IN_PAD,    // external scan_in pins
    //output [5:0] SCAN_OUT_PAD,   // external scan_out pins
    //input  SCAN_ENABLE_PAD,      // external scan_enable
    //input  MODE_PAD,             // external mode signal

    // UART pad signals
    output O_UART_TX_PAD,        // external UART TX
    input  I_UART_RX_PAD,        // external UART RX
    // PWM pad signals
    // output O_PWM_PAD,            // external PWM pad

    // 32‐bit GPIO pads (bidirectional)
    inout  [23:0] IO_DATA_PAD,  // external GPIO pads

    //JTAG pad signals
    input I_TCK_PAD, // external JTAG TCK
    input I_TMS_PAD, // external JTAG TMS
    input I_TDI_PAD, // external JTAG TDI
    output O_TDO_PAD // external JTAG TDO
  );

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
  // Attach the 32-bit GPIO signals
  //-------------------------------------------------------------------------

  wire [23:0] gpio_input_internal;
  wire [23:0] gpio_output_internal;
  wire [23:0] gpio_enable_internal;
  // The internal net "gpio_internal" will connect to the chip’s io_data port.
  // Here we instantiate one pad per bit.
  genvar k;
  generate
    for (k = 0; k < 24; k = k + 1) begin : gpio_pad_gen
      PDD24DGZ u_gpio_pad (
          .I   (gpio_output_internal[k]),
          .OEN (gpio_enable_internal[k]),      // for example, force output (modify as needed)
          .PAD (IO_DATA_PAD[k]),
          .C   (gpio_input_internal[k])           // not used internally in this example
      );
    end
  endgenerate



  wire tck_i_internal;
  PDD24DGZ u_tck_pad (
      .I   (1'b0),
      .OEN (1'b1),
      .PAD (I_TCK_PAD),
      .C   (tck_i_internal)
  );
  wire tms_i_internal;
  PDD24DGZ u_tms_pad (
      .I   (1'b0),
      .OEN (1'b1),
      .PAD (I_TMS_PAD),
      .C   (tms_i_internal)
  );
  wire tdi_i_internal;
  PDD24DGZ u_tdi_pad (
      .I   (1'b0),
      .OEN (1'b1),
      .PAD (I_TDI_PAD),
      .C   (tdi_i_internal)
  );
  wire tdo_o_internal;
  PDD24DGZ u_tdo_pad (
      .I   (tdo_o_internal),
      .OEN (1'b0),
      .PAD (O_TDO_PAD),
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

      // UART signals
      .o_uart_tx    (o_uart_tx_internal),
      .i_uart_rx    (i_uart_rx_internal),

      // 32-bit GPIO (bidirectional)
      .i_gpio       (gpio_input_internal),
      .o_gpio       (gpio_output_internal),
      .en_gpio      (gpio_enable_internal),
    //   .pwm_pad_o    (pwm_pad_o_internal)

      // JTAG signals
      .tck_i        (tck_i_internal),
      .tms_i        (tms_i_internal),
      .tdi_i        (tdi_i_internal),
      .tdo_o        (tdo_o_internal)

  );

endmodule
