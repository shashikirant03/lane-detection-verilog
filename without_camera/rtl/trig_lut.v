// ---------------------------------------------------------------
// trig_lut.v  -- constant table: cos, sin and CORDIC gain for every theta step
// NTH=101  theta step = atan(2^-5) = 1.7899 deg
// ---------------------------------------------------------------
module trig_lut (
    input      [6:0]         k,
    output reg signed [17:0] ginv_q15,
    output reg signed [15:0] cos_q14,
    output reg signed [15:0] sin_q14
);
always @(*) begin
    case (k)
    7'd0  : begin ginv_q15 = 18'sd32768; cos_q14 = 16'sd16384; sin_q14 = 16'sd0; end
    7'd1  : begin ginv_q15 = 18'sd32752; cos_q14 = 16'sd16376; sin_q14 = 16'sd512; end
    7'd2  : begin ginv_q15 = 18'sd32736; cos_q14 = 16'sd16352; sin_q14 = 16'sd1023; end
    7'd3  : begin ginv_q15 = 18'sd32720; cos_q14 = 16'sd16312; sin_q14 = 16'sd1533; end
    7'd4  : begin ginv_q15 = 18'sd32704; cos_q14 = 16'sd16256; sin_q14 = 16'sd2042; end
    7'd5  : begin ginv_q15 = 18'sd32688; cos_q14 = 16'sd16185; sin_q14 = 16'sd2549; end
    7'd6  : begin ginv_q15 = 18'sd32672; cos_q14 = 16'sd16097; sin_q14 = 16'sd3053; end
    7'd7  : begin ginv_q15 = 18'sd32656; cos_q14 = 16'sd15994; sin_q14 = 16'sd3554; end
    7'd8  : begin ginv_q15 = 18'sd32640; cos_q14 = 16'sd15875; sin_q14 = 16'sd4052; end
    7'd9  : begin ginv_q15 = 18'sd32624; cos_q14 = 16'sd15741; sin_q14 = 16'sd4546; end
    7'd10 : begin ginv_q15 = 18'sd32608; cos_q14 = 16'sd15591; sin_q14 = 16'sd5035; end
    7'd11 : begin ginv_q15 = 18'sd32593; cos_q14 = 16'sd15426; sin_q14 = 16'sd5520; end
    7'd12 : begin ginv_q15 = 18'sd32577; cos_q14 = 16'sd15246; sin_q14 = 16'sd5999; end
    7'd13 : begin ginv_q15 = 18'sd32561; cos_q14 = 16'sd15051; sin_q14 = 16'sd6472; end
    7'd14 : begin ginv_q15 = 18'sd32545; cos_q14 = 16'sd14842; sin_q14 = 16'sd6939; end
    7'd15 : begin ginv_q15 = 18'sd32529; cos_q14 = 16'sd14618; sin_q14 = 16'sd7400; end
    7'd16 : begin ginv_q15 = 18'sd32513; cos_q14 = 16'sd14380; sin_q14 = 16'sd7853; end
    7'd17 : begin ginv_q15 = 18'sd32497; cos_q14 = 16'sd14127; sin_q14 = 16'sd8298; end
    7'd18 : begin ginv_q15 = 18'sd32481; cos_q14 = 16'sd13861; sin_q14 = 16'sd8735; end
    7'd19 : begin ginv_q15 = 18'sd32466; cos_q14 = 16'sd13582; sin_q14 = 16'sd9164; end
    7'd20 : begin ginv_q15 = 18'sd32450; cos_q14 = 16'sd13289; sin_q14 = 16'sd9584; end
    7'd21 : begin ginv_q15 = 18'sd32434; cos_q14 = 16'sd12983; sin_q14 = 16'sd9994; end
    7'd22 : begin ginv_q15 = 18'sd32418; cos_q14 = 16'sd12664; sin_q14 = 16'sd10395; end
    7'd23 : begin ginv_q15 = 18'sd32402; cos_q14 = 16'sd12334; sin_q14 = 16'sd10785; end
    7'd24 : begin ginv_q15 = 18'sd32386; cos_q14 = 16'sd11991; sin_q14 = 16'sd11165; end
    7'd25 : begin ginv_q15 = 18'sd32371; cos_q14 = 16'sd11636; sin_q14 = 16'sd11534; end
    7'd26 : begin ginv_q15 = 18'sd32355; cos_q14 = 16'sd11270; sin_q14 = 16'sd11892; end
    7'd27 : begin ginv_q15 = 18'sd32339; cos_q14 = 16'sd10893; sin_q14 = 16'sd12238; end
    7'd28 : begin ginv_q15 = 18'sd32323; cos_q14 = 16'sd10506; sin_q14 = 16'sd12572; end
    7'd29 : begin ginv_q15 = 18'sd32307; cos_q14 = 16'sd10108; sin_q14 = 16'sd12894; end
    7'd30 : begin ginv_q15 = 18'sd32292; cos_q14 = 16'sd9700; sin_q14 = 16'sd13204; end
    7'd31 : begin ginv_q15 = 18'sd32276; cos_q14 = 16'sd9283; sin_q14 = 16'sd13500; end
    7'd32 : begin ginv_q15 = 18'sd32260; cos_q14 = 16'sd8857; sin_q14 = 16'sd13784; end
    7'd33 : begin ginv_q15 = 18'sd32244; cos_q14 = 16'sd8422; sin_q14 = 16'sd14054; end
    7'd34 : begin ginv_q15 = 18'sd32229; cos_q14 = 16'sd7979; sin_q14 = 16'sd14310; end
    7'd35 : begin ginv_q15 = 18'sd32213; cos_q14 = 16'sd7528; sin_q14 = 16'sd14552; end
    7'd36 : begin ginv_q15 = 18'sd32197; cos_q14 = 16'sd7070; sin_q14 = 16'sd14780; end
    7'd37 : begin ginv_q15 = 18'sd32182; cos_q14 = 16'sd6605; sin_q14 = 16'sd14994; end
    7'd38 : begin ginv_q15 = 18'sd32166; cos_q14 = 16'sd6133; sin_q14 = 16'sd15193; end
    7'd39 : begin ginv_q15 = 18'sd32150; cos_q14 = 16'sd5656; sin_q14 = 16'sd15377; end
    7'd40 : begin ginv_q15 = 18'sd32135; cos_q14 = 16'sd5173; sin_q14 = 16'sd15546; end
    7'd41 : begin ginv_q15 = 18'sd32119; cos_q14 = 16'sd4684; sin_q14 = 16'sd15700; end
    7'd42 : begin ginv_q15 = 18'sd32103; cos_q14 = 16'sd4192; sin_q14 = 16'sd15839; end
    7'd43 : begin ginv_q15 = 18'sd32088; cos_q14 = 16'sd3695; sin_q14 = 16'sd15962; end
    7'd44 : begin ginv_q15 = 18'sd32072; cos_q14 = 16'sd3195; sin_q14 = 16'sd16070; end
    7'd45 : begin ginv_q15 = 18'sd32056; cos_q14 = 16'sd2691; sin_q14 = 16'sd16161; end
    7'd46 : begin ginv_q15 = 18'sd32041; cos_q14 = 16'sd2185; sin_q14 = 16'sd16238; end
    7'd47 : begin ginv_q15 = 18'sd32025; cos_q14 = 16'sd1677; sin_q14 = 16'sd16298; end
    7'd48 : begin ginv_q15 = 18'sd32009; cos_q14 = 16'sd1167; sin_q14 = 16'sd16342; end
    7'd49 : begin ginv_q15 = 18'sd31994; cos_q14 = 16'sd656; sin_q14 = 16'sd16371; end
    7'd50 : begin ginv_q15 = 18'sd31978; cos_q14 = 16'sd144; sin_q14 = 16'sd16383; end
    7'd51 : begin ginv_q15 = 18'sd31962; cos_q14 = -16'sd368; sin_q14 = 16'sd16380; end
    7'd52 : begin ginv_q15 = 18'sd31947; cos_q14 = -16'sd879; sin_q14 = 16'sd16360; end
    7'd53 : begin ginv_q15 = 18'sd31931; cos_q14 = -16'sd1390; sin_q14 = 16'sd16325; end
    7'd54 : begin ginv_q15 = 18'sd31916; cos_q14 = -16'sd1899; sin_q14 = 16'sd16274; end
    7'd55 : begin ginv_q15 = 18'sd31900; cos_q14 = -16'sd2406; sin_q14 = 16'sd16206; end
    7'd56 : begin ginv_q15 = 18'sd31885; cos_q14 = -16'sd2911; sin_q14 = 16'sd16123; end
    7'd57 : begin ginv_q15 = 18'sd31869; cos_q14 = -16'sd3413; sin_q14 = 16'sd16024; end
    7'd58 : begin ginv_q15 = 18'sd31853; cos_q14 = -16'sd3912; sin_q14 = 16'sd15910; end
    7'd59 : begin ginv_q15 = 18'sd31838; cos_q14 = -16'sd4407; sin_q14 = 16'sd15780; end
    7'd60 : begin ginv_q15 = 18'sd31822; cos_q14 = -16'sd4898; sin_q14 = 16'sd15635; end
    7'd61 : begin ginv_q15 = 18'sd31807; cos_q14 = -16'sd5384; sin_q14 = 16'sd15474; end
    7'd62 : begin ginv_q15 = 18'sd31791; cos_q14 = -16'sd5865; sin_q14 = 16'sd15298; end
    7'd63 : begin ginv_q15 = 18'sd31776; cos_q14 = -16'sd6340; sin_q14 = 16'sd15108; end
    7'd64 : begin ginv_q15 = 18'sd31760; cos_q14 = -16'sd6808; sin_q14 = 16'sd14902; end
    7'd65 : begin ginv_q15 = 18'sd31745; cos_q14 = -16'sd7271; sin_q14 = 16'sd14682; end
    7'd66 : begin ginv_q15 = 18'sd31729; cos_q14 = -16'sd7726; sin_q14 = 16'sd14448; end
    7'd67 : begin ginv_q15 = 18'sd31714; cos_q14 = -16'sd8173; sin_q14 = 16'sd14200; end
    7'd68 : begin ginv_q15 = 18'sd31698; cos_q14 = -16'sd8613; sin_q14 = 16'sd13938; end
    7'd69 : begin ginv_q15 = 18'sd31683; cos_q14 = -16'sd9044; sin_q14 = 16'sd13662; end
    7'd70 : begin ginv_q15 = 18'sd31667; cos_q14 = -16'sd9466; sin_q14 = 16'sd13373; end
    7'd71 : begin ginv_q15 = 18'sd31652; cos_q14 = -16'sd9879; sin_q14 = 16'sd13070; end
    7'd72 : begin ginv_q15 = 18'sd31637; cos_q14 = -16'sd10283; sin_q14 = 16'sd12755; end
    7'd73 : begin ginv_q15 = 18'sd31621; cos_q14 = -16'sd10676; sin_q14 = 16'sd12428; end
    7'd74 : begin ginv_q15 = 18'sd31606; cos_q14 = -16'sd11059; sin_q14 = 16'sd12089; end
    7'd75 : begin ginv_q15 = 18'sd31590; cos_q14 = -16'sd11431; sin_q14 = 16'sd11737; end
    7'd76 : begin ginv_q15 = 18'sd31575; cos_q14 = -16'sd11792; sin_q14 = 16'sd11374; end
    7'd77 : begin ginv_q15 = 18'sd31559; cos_q14 = -16'sd12142; sin_q14 = 16'sd11001; end
    7'd78 : begin ginv_q15 = 18'sd31544; cos_q14 = -16'sd12479; sin_q14 = 16'sd10616; end
    7'd79 : begin ginv_q15 = 18'sd31529; cos_q14 = -16'sd12805; sin_q14 = 16'sd10221; end
    7'd80 : begin ginv_q15 = 18'sd31513; cos_q14 = -16'sd13118; sin_q14 = 16'sd9816; end
    7'd81 : begin ginv_q15 = 18'sd31498; cos_q14 = -16'sd13418; sin_q14 = 16'sd9402; end
    7'd82 : begin ginv_q15 = 18'sd31483; cos_q14 = -16'sd13705; sin_q14 = 16'sd8978; end
    7'd83 : begin ginv_q15 = 18'sd31467; cos_q14 = -16'sd13979; sin_q14 = 16'sd8545; end
    7'd84 : begin ginv_q15 = 18'sd31452; cos_q14 = -16'sd14239; sin_q14 = 16'sd8105; end
    7'd85 : begin ginv_q15 = 18'sd31436; cos_q14 = -16'sd14485; sin_q14 = 16'sd7656; end
    7'd86 : begin ginv_q15 = 18'sd31421; cos_q14 = -16'sd14717; sin_q14 = 16'sd7200; end
    7'd87 : begin ginv_q15 = 18'sd31406; cos_q14 = -16'sd14935; sin_q14 = 16'sd6736; end
    7'd88 : begin ginv_q15 = 18'sd31390; cos_q14 = -16'sd15138; sin_q14 = 16'sd6267; end
    7'd89 : begin ginv_q15 = 18'sd31375; cos_q14 = -16'sd15327; sin_q14 = 16'sd5791; end
    7'd90 : begin ginv_q15 = 18'sd31360; cos_q14 = -16'sd15500; sin_q14 = 16'sd5309; end
    7'd91 : begin ginv_q15 = 18'sd31345; cos_q14 = -16'sd15658; sin_q14 = 16'sd4823; end
    7'd92 : begin ginv_q15 = 18'sd31329; cos_q14 = -16'sd15801; sin_q14 = 16'sd4331; end
    7'd93 : begin ginv_q15 = 18'sd31314; cos_q14 = -16'sd15929; sin_q14 = 16'sd3835; end
    7'd94 : begin ginv_q15 = 18'sd31299; cos_q14 = -16'sd16041; sin_q14 = 16'sd3336; end
    7'd95 : begin ginv_q15 = 18'sd31283; cos_q14 = -16'sd16137; sin_q14 = 16'sd2833; end
    7'd96 : begin ginv_q15 = 18'sd31268; cos_q14 = -16'sd16218; sin_q14 = 16'sd2328; end
    7'd97 : begin ginv_q15 = 18'sd31253; cos_q14 = -16'sd16283; sin_q14 = 16'sd1820; end
    7'd98 : begin ginv_q15 = 18'sd31238; cos_q14 = -16'sd16331; sin_q14 = 16'sd1311; end
    7'd99 : begin ginv_q15 = 18'sd31222; cos_q14 = -16'sd16364; sin_q14 = 16'sd800; end
    7'd100: begin ginv_q15 = 18'sd31207; cos_q14 = -16'sd16381; sin_q14 = 16'sd288; end
    default: begin ginv_q15 = 18'sd0; cos_q14 = 16'sd0; sin_q14 = 16'sd0; end
    endcase
end
endmodule
