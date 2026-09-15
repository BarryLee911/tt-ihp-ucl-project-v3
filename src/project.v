module tt_um_sine_area_detector #(
    parameter integer HANDOFF_WAIT_CYCLES = 80000
) (

    input  wire [7:0] ui_in,//8-bit ADC code

    output wire [7:0] uo_out,//Low 8 bits of area or peak

    input  wire [7:0] uio_in,//uio[4:0]: level before latching

    output wire [7:0] uio_out,//uio[7:5]: upper result bits; uio[0]: result type

    output wire [7:0] uio_oe,//uio[0] becomes an output after handoff

    /* Retained for template compatibility; operation ignores ena. */
    input  wire       ena,

    input  wire       clk,//80 MHz

    input  wire       rst_n
);

    wire _unused = &{ena, 1'b0};

    /* Convert the ADC code to a binary sign. */
    wire adc_sign;
    assign adc_sign = (ui_in >= 8'h80);

    /* Levels 0-22: binary division; level 23: 0.5 mHz at 80 MHz. */
    reg [4:0] divider_exponent;
    reg       config_valid;

    always @* begin
        divider_exponent = uio_in[4:0];
        config_valid     = 1'b0;

        if (uio_in[4:0] <= 5'd23)
            config_valid = 1'b1;
    end

    wire overlap_bit;
    assign overlap_bit = adc_sign == square_wave;

    /* Latch the first valid level after reset. */
    reg       config_latched_valid;
    reg [4:0] divider_exponent_latched;

    /*
     * Sample every 2^level clocks; level 23 uses 78,125,000 clocks.
     */
    reg  [26:0] prescale_count;
    wire [26:0] prescale_terminal_extended;
    wire        sample_tick;

    assign prescale_terminal_extended =
        (divider_exponent_latched == 5'd23) ? 27'd78124999 :
        (27'd1 << divider_exponent_latched) - 27'd1;
    assign sample_tick =
        (prescale_count == prescale_terminal_extended);

    /* Count ones in the last 2048 overlap samples. */
    reg        overlap_history [0:2047];
    reg [10:0] history_pointer;
    reg [11:0] running_sum;
    reg        window_full;
    /* Independent square wave */
    wire square_wave;
    assign square_wave = history_pointer[10];

    wire        oldest_overlap;
    wire        effective_oldest_overlap;
    wire [11:0] slide_sum_next;

    assign oldest_overlap = overlap_history[history_pointer];
    /* Unwritten history counts as zero. */
    assign effective_oldest_overlap = window_full ? oldest_overlap : 1'b0;
    assign slide_sum_next =
        ( overlap_bit && !effective_oldest_overlap) ? (running_sum + 12'd1) :
        (!overlap_bit &&  effective_oldest_overlap) ? (running_sum - 12'd1) :
                                            running_sum;

    /* Absolute ADC distance from middle. */
    wire [7:0] adc_magnitude;
    assign adc_magnitude = ui_in[7] ? (ui_in - 8'd128) : (8'd128 - ui_in);

    /* Two peak candidates and the latest sample. */
    reg [7:0] peak_first;
    reg [7:0] peak_second;
    reg [7:0] peak_buffer;
    reg [9:0] peak_first_position;
    reg [9:0] peak_second_position;
    reg [9:0] peak_buffer_position;
    reg peak_first_valid;
    reg peak_second_valid;
    reg peak_buffer_valid;

    reg [7:0] peak_first_next;
    reg [7:0] peak_second_next;
    reg [9:0] peak_first_position_next;
    reg [9:0] peak_second_position_next;
    reg peak_first_valid_next;
    reg peak_second_valid_next;
    wire [7:0] peak_buffer_next;
    wire [7:0] peak_next;
    wire take_sample;
    assign take_sample = config_latched_valid && sample_tick;
    assign peak_buffer_next = adc_magnitude;

    always @* begin
        peak_first_next = peak_first;
        peak_second_next = peak_second;
        peak_first_position_next = peak_first_position;
        peak_second_position_next = peak_second_position;

        /* Expire samples after 1024 sampling steps. */
        peak_first_valid_next = peak_first_valid &&
            (peak_first_position != history_pointer[9:0]);
        peak_second_valid_next = peak_second_valid &&
            (peak_second_position != history_pointer[9:0]);

        /* Promote second without changing its age. */
        if (!peak_first_valid_next) begin
            peak_first_next = peak_second_next;
            peak_first_position_next = peak_second_position_next;
            peak_first_valid_next = peak_second_valid_next;
            peak_second_valid_next = 1'b0;
        end

        /* Buffer is one sample old and has already competed with both peaks. */
        /* Fill second without duplicating the surviving first sample. */
        if (!peak_second_valid_next && peak_buffer_valid &&
            (!peak_first_valid_next ||
             (peak_buffer_position != peak_first_position_next))) begin
            peak_second_next = peak_buffer;
            peak_second_position_next = peak_buffer_position;
            peak_second_valid_next = 1'b1;
        end

        /* Compare the incoming sample; newer ties win. */
        if (!peak_first_valid_next || (peak_buffer_next >= peak_first_next)) begin
            peak_second_next = peak_first_next;
            peak_second_position_next = peak_first_position_next;
            peak_second_valid_next = peak_first_valid_next;
            peak_first_next = peak_buffer_next;
            peak_first_position_next = history_pointer[9:0];
            peak_first_valid_next = 1'b1;
        end else if (!peak_second_valid_next || (peak_buffer_next >= peak_second_next)) begin
            peak_second_next = peak_buffer_next;
            peak_second_position_next = history_pointer[9:0];
            peak_second_valid_next = 1'b1;
        end
    end
    assign peak_next = peak_first_next;

    /* Wait 1 ms at 80 MHz before driving uio[0]. */
    localparam integer HANDOFF_CYCLES =
        (HANDOFF_WAIT_CYCLES < 1) ? 1 : HANDOFF_WAIT_CYCLES;
    /* Up to 131072 clocks. */
    localparam [16:0] HANDOFF_LAST = HANDOFF_CYCLES - 1;
    reg [16:0] handoff_count;
    reg output_ready;
    /* Register data and its type on the same edge. */
    wire [11:0] area_value;
    wire [10:0] area_latest;
    wire [7:0] peak_latest;
    reg [10:0] output_data;
    reg output_kind;
    wire send_peak;
    assign area_value = take_sample ? slide_sum_next : running_sum;
    assign area_latest = area_value[11] ? 11'd2047 : area_value[10:0];
    assign send_peak = output_ready && !output_kind;
    assign peak_latest = take_sample ? peak_next : peak_first;
    /* One synchronous reset; each section keeps its own enable conditions. */
    always @(posedge clk) begin
        if (!rst_n) begin
            config_latched_valid     <= 1'b0;
            divider_exponent_latched <= 5'd0;
            prescale_count           <= 27'd0;
            history_pointer          <= 11'd0;
            running_sum              <= 12'd0;
            window_full              <= 1'b0;

            peak_first <= 8'd0;
            peak_second <= 8'd0;
            peak_buffer <= 8'd0;
            peak_first_position <= 10'd0;
            peak_second_position <= 10'd0;
            peak_buffer_position <= 10'd0;
            peak_first_valid <= 1'b0;
            peak_second_valid <= 1'b0;
            peak_buffer_valid <= 1'b0;

            handoff_count <= 0;
            output_ready  <= 1'b0;

            output_data <= 11'd0;
            output_kind <= 1'b0;
        end else begin
            /* Hold reset state until a valid level is latched. */
            if (!config_latched_valid && config_valid) begin
                divider_exponent_latched <= divider_exponent;
                config_latched_valid <= 1'b1;
            end

            if (config_latched_valid) begin
                prescale_count <= sample_tick ? 27'd0 : prescale_count + 27'd1;
            end

            if (take_sample) begin
                /* Replace the oldest sample, then advance the pointer. */
                overlap_history[history_pointer] <= overlap_bit;
                history_pointer <= history_pointer + 11'd1;
                if (history_pointer == 11'd2047)
                    window_full <= 1'b1;//check
                running_sum <= slide_sum_next;

                peak_first <= peak_first_next;
                peak_second <= peak_second_next;
                peak_buffer <= peak_buffer_next;
                peak_first_position <= peak_first_position_next;
                peak_second_position <= peak_second_position_next;
                peak_buffer_position <= history_pointer[9:0];
                peak_first_valid <= peak_first_valid_next;
                peak_second_valid <= peak_second_valid_next;
                peak_buffer_valid <= 1'b1;
            end

            if (config_latched_valid && !output_ready) begin
                if (handoff_count == HANDOFF_LAST)
                    output_ready <= 1'b1;
                else
                    handoff_count <= handoff_count + 1'b1;
            end

            output_kind <= send_peak;
            output_data <= send_peak ? {3'b000, peak_latest} : area_latest;
        end
    end

    /* Type 0: area; type 1: peak. */
    assign uo_out  = output_data[7:0];
    assign uio_out = {output_data[10:8], 4'b0000, output_kind};
    assign uio_oe  = output_ready ? 8'he1 : 8'he0;

endmodule
