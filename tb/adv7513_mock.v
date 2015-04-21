module adv7513_mock #(
        parameter CHIP_ADDR = 7'h72
    )(
        input clock,
        input reset,
        inout SDA,
        inout SCL
    );

    reg  [6:0]  slave_chip_addr;
    wire [7:0]  slave_reg_addr;

    wire        slave_busy;
    wire        slave_done;

    wire        slave_write_en;
    reg  [7:0] slave_data_in;
    wire [7:0] slave_data_out;

    wire        slave_sda_in;
    wire        slave_scl_in;

    wire        slave_sda_out;
    wire        slave_sda_oen;
    wire        slave_scl_out;
    wire        slave_scl_oen;

    // For storing slave data
    reg [7:0]  slave_data[0:255];


    // SDA Input / Output
    assign slave_sda_in = sda;
    assign SDA = (slave_sda_oen == 0) ? slave_sda_out : 1'bz;

    // SCL Input / Output
    assign slave_scl_in = scl;
    assign SCL = (slave_scl_oen == 0) ? slave_scl_out : 1'bz;

    pullup(SDA);
    pullup(SCL);

    // i2c Slave
    i2c_slave #(
        .ADDR_BYTES(1),
        .DATA_BYTES(1)
    ) i2c_slave (
        .clk        (clock),
        .reset      (reset),

        .open_drain (1'b1),

        .chip_addr  (CHIP_ADDR),
        .reg_addr   (slave_reg_addr),
        .data_in    (slave_data_in),
        .write_en   (slave_write_en),
        .data_out   (slave_data_out),
        .done       (slave_done),
        .busy       (slave_busy),

        .sda_in     (slave_sda_in),
        .scl_in     (slave_scl_in),
        .sda_out    (slave_sda_out),
        .sda_oen    (slave_sda_oen),
        .scl_out    (slave_scl_out),
        .scl_oen    (slave_scl_oen)
    );

    // Save slave data to register
    always @ (posedge clock) begin
        if (slave_write_en) begin
            slave_data[slave_reg_addr] <= slave_data_out;
        end

        slave_data_in <= slave_data[slave_reg_addr];
    end
endmodule