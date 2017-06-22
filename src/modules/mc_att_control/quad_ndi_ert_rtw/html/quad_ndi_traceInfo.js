function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/vehicle_attitude */
	this.urlHashMap["quad_ndi:396"] = "ert_main.cpp:24&quad_ndi.cpp:44,57,75,93,109,117,159,174";
	/* <Root>/vehicle_attitude_setpoint */
	this.urlHashMap["quad_ndi:397"] = "ert_main.cpp:27&quad_ndi.cpp:45,76,110,175";
	/* <Root>/param */
	this.urlHashMap["quad_ndi:398"] = "ert_main.cpp:30&quad_ndi.cpp:43,56,66,74,86,92,102,116,126,136,151";
	/* <Root>/Constant */
	this.urlHashMap["quad_ndi:393"] = "msg=rtwMsg_notTraceable&block=quad_ndi:393";
	/* <Root>/Display1 */
	this.urlHashMap["quad_ndi:822"] = "quad_ndi.h:106";
	/* <Root>/Display2 */
	this.urlHashMap["quad_ndi:823"] = "quad_ndi.h:107";
	/* <Root>/Display3 */
	this.urlHashMap["quad_ndi:827"] = "msg=rtwMsg_notTraceable&block=quad_ndi:827";
	/* <Root>/Display4 */
	this.urlHashMap["quad_ndi:828"] = "msg=rtwMsg_notTraceable&block=quad_ndi:828";
	/* <Root>/Gain1 */
	this.urlHashMap["quad_ndi:815"] = "quad_ndi.cpp:187&quad_ndi.h:49&quad_ndi_data.cpp:22";
	/* <Root>/Gain2 */
	this.urlHashMap["quad_ndi:846"] = "quad_ndi.cpp:173";
	/* <Root>/MATLAB Function */
	this.urlHashMap["quad_ndi:640"] = "quad_ndi.h:128";
	/* <Root>/MATLAB Function1 */
	this.urlHashMap["quad_ndi:818"] = "quad_ndi.h:129";
	/* <Root>/NDI LAW */
	this.urlHashMap["quad_ndi:683"] = "quad_ndi.cpp:158,176&quad_ndi.h:122,130";
	/* <Root>/Product2 */
	this.urlHashMap["quad_ndi:106"] = "msg=rtwMsg_notTraceable&block=quad_ndi:106";
	/* <Root>/Product3 */
	this.urlHashMap["quad_ndi:389"] = "msg=rtwMsg_notTraceable&block=quad_ndi:389";
	/* <Root>/Product4 */
	this.urlHashMap["quad_ndi:390"] = "msg=rtwMsg_notTraceable&block=quad_ndi:390";
	/* <Root>/Scope */
	this.urlHashMap["quad_ndi:661"] = "msg=rtwMsg_notTraceable&block=quad_ndi:661";
	/* <Root>/Scope1 */
	this.urlHashMap["quad_ndi:814"] = "quad_ndi.h:108";
	/* <Root>/Subsystem */
	this.urlHashMap["quad_ndi:381"] = "quad_ndi.h:131";
	/* <Root>/Subsystem1 */
	this.urlHashMap["quad_ndi:690"] = "quad_ndi.h:132";
	/* <Root>/angle_control */
	this.urlHashMap["quad_ndi:638"] = "quad_ndi.h:133";
	/* <Root>/control_attitude */
	this.urlHashMap["quad_ndi:696"] = "quad_ndi.cpp:41,52,72,83&quad_ndi.h:134";
	/* <Root>/control_attitude_rates */
	this.urlHashMap["quad_ndi:713"] = "quad_ndi.cpp:40,156&quad_ndi.h:135";
	/* <Root>/min_pwm */
	this.urlHashMap["quad_ndi:711"] = "msg=rtwMsg_notTraceable&block=quad_ndi:711";
	/* <Root>/mixer */
	this.urlHashMap["quad_ndi:639"] = "quad_ndi.h:136";
	/* <Root>/rate_control */
	this.urlHashMap["quad_ndi:634"] = "quad_ndi.h:137";
	/* <Root>/cmd */
	this.urlHashMap["quad_ndi:831"] = "ert_main.cpp:34&quad_ndi.cpp:186,198";
	/* <S1>:1 */
	this.urlHashMap["quad_ndi:640:1"] = "msg=rtwMsg_notTraceable&block=quad_ndi:640:1";
	/* <S2>:1 */
	this.urlHashMap["quad_ndi:818:1"] = "msg=rtwMsg_notTraceable&block=quad_ndi:818:1";
	/* <S3>:1 */
	this.urlHashMap["quad_ndi:683:1"] = "quad_ndi.cpp:162";
	/* <S3>:1:4 */
	this.urlHashMap["quad_ndi:683:1:4"] = "quad_ndi.cpp:163,165";
	/* <S6>:1 */
	this.urlHashMap["quad_ndi:638:1"] = "msg=rtwMsg_notTraceable&block=quad_ndi:638:1";
	/* <S7>/PID Controller1 */
	this.urlHashMap["quad_ndi:704"] = "quad_ndi.h:138";
	/* <S7>/PID Controller2 */
	this.urlHashMap["quad_ndi:705"] = "quad_ndi.h:139";
	/* <S7>/PID Controller3 */
	this.urlHashMap["quad_ndi:706"] = "quad_ndi.h:140";
	/* <S7>/Sum1 */
	this.urlHashMap["quad_ndi:707"] = "quad_ndi.h:110";
	/* <S7>/Sum5 */
	this.urlHashMap["quad_ndi:708"] = "quad_ndi.cpp:47";
	/* <S7>/Sum6 */
	this.urlHashMap["quad_ndi:709"] = "quad_ndi.cpp:78";
	/* <S7>/euler_rates_2_body_rates */
	this.urlHashMap["quad_ndi:830"] = "quad_ndi.h:141";
	/* <S8>/Discrete_PID_measurement_D */
	this.urlHashMap["quad_ndi:720"] = "quad_ndi.h:142";
	/* <S8>/Discrete_PID_measurement_D1 */
	this.urlHashMap["quad_ndi:737"] = "quad_ndi.h:143";
	/* <S8>/Discrete_PID_measurement_D2 */
	this.urlHashMap["quad_ndi:754"] = "quad_ndi.h:144";
	/* <S9>:1 */
	this.urlHashMap["quad_ndi:639:1"] = "msg=rtwMsg_notTraceable&block=quad_ndi:639:1";
	/* <S10>:1 */
	this.urlHashMap["quad_ndi:634:1"] = "msg=rtwMsg_notTraceable&block=quad_ndi:634:1";
	/* <S11>/POut */
	this.urlHashMap["quad_ndi:704:1656"] = "quad_ndi.cpp:77";
	/* <S12>/POut */
	this.urlHashMap["quad_ndi:705:1656"] = "quad_ndi.cpp:46";
	/* <S13>/POut */
	this.urlHashMap["quad_ndi:706:1656"] = "quad_ndi.h:109";
	/* <S14>:1 */
	this.urlHashMap["quad_ndi:830:1"] = "msg=rtwMsg_notTraceable&block=quad_ndi:830:1";
	/* <S15>/Dout */
	this.urlHashMap["quad_ndi:727"] = "quad_ndi.cpp:58";
	/* <S15>/Filter State */
	this.urlHashMap["quad_ndi:728"] = "quad_ndi.cpp:55,132&quad_ndi.h:38";
	/* <S15>/Integrator */
	this.urlHashMap["quad_ndi:729"] = "quad_ndi.cpp:65,135&quad_ndi.h:39";
	/* <S15>/Iout */
	this.urlHashMap["quad_ndi:730"] = "quad_ndi.cpp:137";
	/* <S15>/NOut */
	this.urlHashMap["quad_ndi:731"] = "quad_ndi.cpp:54";
	/* <S15>/Pout */
	this.urlHashMap["quad_ndi:732"] = "quad_ndi.cpp:67";
	/* <S15>/Sum1 */
	this.urlHashMap["quad_ndi:733"] = "quad_ndi.cpp:59";
	/* <S15>/Sum2 */
	this.urlHashMap["quad_ndi:734"] = "quad_ndi.cpp:64";
	/* <S15>/error */
	this.urlHashMap["quad_ndi:735"] = "quad_ndi.cpp:42";
	/* <S16>/Dout */
	this.urlHashMap["quad_ndi:744"] = "quad_ndi.cpp:94";
	/* <S16>/Filter State */
	this.urlHashMap["quad_ndi:745"] = "quad_ndi.cpp:91,141&quad_ndi.h:40";
	/* <S16>/Integrator */
	this.urlHashMap["quad_ndi:746"] = "quad_ndi.cpp:101,144&quad_ndi.h:41";
	/* <S16>/Iout */
	this.urlHashMap["quad_ndi:747"] = "quad_ndi.cpp:85";
	/* <S16>/NOut */
	this.urlHashMap["quad_ndi:748"] = "quad_ndi.cpp:90";
	/* <S16>/Pout */
	this.urlHashMap["quad_ndi:749"] = "quad_ndi.cpp:103";
	/* <S16>/Sum1 */
	this.urlHashMap["quad_ndi:750"] = "quad_ndi.cpp:95";
	/* <S16>/Sum2 */
	this.urlHashMap["quad_ndi:751"] = "quad_ndi.cpp:100";
	/* <S16>/error */
	this.urlHashMap["quad_ndi:752"] = "quad_ndi.cpp:73";
	/* <S17>/Dout */
	this.urlHashMap["quad_ndi:761"] = "quad_ndi.cpp:118";
	/* <S17>/Filter State */
	this.urlHashMap["quad_ndi:762"] = "quad_ndi.cpp:115,147&quad_ndi.h:42";
	/* <S17>/Integrator */
	this.urlHashMap["quad_ndi:763"] = "quad_ndi.cpp:125,150&quad_ndi.h:43";
	/* <S17>/Iout */
	this.urlHashMap["quad_ndi:764"] = "quad_ndi.cpp:152";
	/* <S17>/NOut */
	this.urlHashMap["quad_ndi:765"] = "quad_ndi.cpp:114";
	/* <S17>/Pout */
	this.urlHashMap["quad_ndi:766"] = "quad_ndi.cpp:127";
	/* <S17>/Sum1 */
	this.urlHashMap["quad_ndi:767"] = "quad_ndi.cpp:119";
	/* <S17>/Sum2 */
	this.urlHashMap["quad_ndi:768"] = "quad_ndi.cpp:124";
	/* <S17>/error */
	this.urlHashMap["quad_ndi:769"] = "quad_ndi.cpp:108";
	this.getUrlHash = function(sid) { return this.urlHashMap[sid];}
}
RTW_Sid2UrlHash.instance = new RTW_Sid2UrlHash();
function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "quad_ndi"};
	this.sidHashMap["quad_ndi"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<S1>"] = {sid: "quad_ndi:640"};
	this.sidHashMap["quad_ndi:640"] = {rtwname: "<S1>"};
	this.rtwnameHashMap["<S2>"] = {sid: "quad_ndi:818"};
	this.sidHashMap["quad_ndi:818"] = {rtwname: "<S2>"};
	this.rtwnameHashMap["<S3>"] = {sid: "quad_ndi:683"};
	this.sidHashMap["quad_ndi:683"] = {rtwname: "<S3>"};
	this.rtwnameHashMap["<S4>"] = {sid: "quad_ndi:381"};
	this.sidHashMap["quad_ndi:381"] = {rtwname: "<S4>"};
	this.rtwnameHashMap["<S5>"] = {sid: "quad_ndi:690"};
	this.sidHashMap["quad_ndi:690"] = {rtwname: "<S5>"};
	this.rtwnameHashMap["<S6>"] = {sid: "quad_ndi:638"};
	this.sidHashMap["quad_ndi:638"] = {rtwname: "<S6>"};
	this.rtwnameHashMap["<S7>"] = {sid: "quad_ndi:696"};
	this.sidHashMap["quad_ndi:696"] = {rtwname: "<S7>"};
	this.rtwnameHashMap["<S8>"] = {sid: "quad_ndi:713"};
	this.sidHashMap["quad_ndi:713"] = {rtwname: "<S8>"};
	this.rtwnameHashMap["<S9>"] = {sid: "quad_ndi:639"};
	this.sidHashMap["quad_ndi:639"] = {rtwname: "<S9>"};
	this.rtwnameHashMap["<S10>"] = {sid: "quad_ndi:634"};
	this.sidHashMap["quad_ndi:634"] = {rtwname: "<S10>"};
	this.rtwnameHashMap["<S11>"] = {sid: "quad_ndi:704"};
	this.sidHashMap["quad_ndi:704"] = {rtwname: "<S11>"};
	this.rtwnameHashMap["<S12>"] = {sid: "quad_ndi:705"};
	this.sidHashMap["quad_ndi:705"] = {rtwname: "<S12>"};
	this.rtwnameHashMap["<S13>"] = {sid: "quad_ndi:706"};
	this.sidHashMap["quad_ndi:706"] = {rtwname: "<S13>"};
	this.rtwnameHashMap["<S14>"] = {sid: "quad_ndi:830"};
	this.sidHashMap["quad_ndi:830"] = {rtwname: "<S14>"};
	this.rtwnameHashMap["<S15>"] = {sid: "quad_ndi:720"};
	this.sidHashMap["quad_ndi:720"] = {rtwname: "<S15>"};
	this.rtwnameHashMap["<S16>"] = {sid: "quad_ndi:737"};
	this.sidHashMap["quad_ndi:737"] = {rtwname: "<S16>"};
	this.rtwnameHashMap["<S17>"] = {sid: "quad_ndi:754"};
	this.sidHashMap["quad_ndi:754"] = {rtwname: "<S17>"};
	this.rtwnameHashMap["<Root>/vehicle_attitude"] = {sid: "quad_ndi:396"};
	this.sidHashMap["quad_ndi:396"] = {rtwname: "<Root>/vehicle_attitude"};
	this.rtwnameHashMap["<Root>/vehicle_attitude_setpoint"] = {sid: "quad_ndi:397"};
	this.sidHashMap["quad_ndi:397"] = {rtwname: "<Root>/vehicle_attitude_setpoint"};
	this.rtwnameHashMap["<Root>/param"] = {sid: "quad_ndi:398"};
	this.sidHashMap["quad_ndi:398"] = {rtwname: "<Root>/param"};
	this.rtwnameHashMap["<Root>/Constant"] = {sid: "quad_ndi:393"};
	this.sidHashMap["quad_ndi:393"] = {rtwname: "<Root>/Constant"};
	this.rtwnameHashMap["<Root>/Demux"] = {sid: "quad_ndi:835"};
	this.sidHashMap["quad_ndi:835"] = {rtwname: "<Root>/Demux"};
	this.rtwnameHashMap["<Root>/Demux4"] = {sid: "quad_ndi:399"};
	this.sidHashMap["quad_ndi:399"] = {rtwname: "<Root>/Demux4"};
	this.rtwnameHashMap["<Root>/Display1"] = {sid: "quad_ndi:822"};
	this.sidHashMap["quad_ndi:822"] = {rtwname: "<Root>/Display1"};
	this.rtwnameHashMap["<Root>/Display2"] = {sid: "quad_ndi:823"};
	this.sidHashMap["quad_ndi:823"] = {rtwname: "<Root>/Display2"};
	this.rtwnameHashMap["<Root>/Display3"] = {sid: "quad_ndi:827"};
	this.sidHashMap["quad_ndi:827"] = {rtwname: "<Root>/Display3"};
	this.rtwnameHashMap["<Root>/Display4"] = {sid: "quad_ndi:828"};
	this.sidHashMap["quad_ndi:828"] = {rtwname: "<Root>/Display4"};
	this.rtwnameHashMap["<Root>/From"] = {sid: "quad_ndi:663"};
	this.sidHashMap["quad_ndi:663"] = {rtwname: "<Root>/From"};
	this.rtwnameHashMap["<Root>/From1"] = {sid: "quad_ndi:365"};
	this.sidHashMap["quad_ndi:365"] = {rtwname: "<Root>/From1"};
	this.rtwnameHashMap["<Root>/From11"] = {sid: "quad_ndi:385"};
	this.sidHashMap["quad_ndi:385"] = {rtwname: "<Root>/From11"};
	this.rtwnameHashMap["<Root>/From14"] = {sid: "quad_ndi:386"};
	this.sidHashMap["quad_ndi:386"] = {rtwname: "<Root>/From14"};
	this.rtwnameHashMap["<Root>/From15"] = {sid: "quad_ndi:387"};
	this.sidHashMap["quad_ndi:387"] = {rtwname: "<Root>/From15"};
	this.rtwnameHashMap["<Root>/From16"] = {sid: "quad_ndi:388"};
	this.sidHashMap["quad_ndi:388"] = {rtwname: "<Root>/From16"};
	this.rtwnameHashMap["<Root>/From2"] = {sid: "quad_ndi:664"};
	this.sidHashMap["quad_ndi:664"] = {rtwname: "<Root>/From2"};
	this.rtwnameHashMap["<Root>/From25"] = {sid: "quad_ndi:108"};
	this.sidHashMap["quad_ndi:108"] = {rtwname: "<Root>/From25"};
	this.rtwnameHashMap["<Root>/From26"] = {sid: "quad_ndi:105"};
	this.sidHashMap["quad_ndi:105"] = {rtwname: "<Root>/From26"};
	this.rtwnameHashMap["<Root>/From4"] = {sid: "quad_ndi:837"};
	this.sidHashMap["quad_ndi:837"] = {rtwname: "<Root>/From4"};
	this.rtwnameHashMap["<Root>/From6"] = {sid: "quad_ndi:842"};
	this.sidHashMap["quad_ndi:842"] = {rtwname: "<Root>/From6"};
	this.rtwnameHashMap["<Root>/From61"] = {sid: "quad_ndi:632"};
	this.sidHashMap["quad_ndi:632"] = {rtwname: "<Root>/From61"};
	this.rtwnameHashMap["<Root>/From62"] = {sid: "quad_ndi:635"};
	this.sidHashMap["quad_ndi:635"] = {rtwname: "<Root>/From62"};
	this.rtwnameHashMap["<Root>/From63"] = {sid: "quad_ndi:636"};
	this.sidHashMap["quad_ndi:636"] = {rtwname: "<Root>/From63"};
	this.rtwnameHashMap["<Root>/From64"] = {sid: "quad_ndi:671"};
	this.sidHashMap["quad_ndi:671"] = {rtwname: "<Root>/From64"};
	this.rtwnameHashMap["<Root>/From67"] = {sid: "quad_ndi:674"};
	this.sidHashMap["quad_ndi:674"] = {rtwname: "<Root>/From67"};
	this.rtwnameHashMap["<Root>/From68"] = {sid: "quad_ndi:675"};
	this.sidHashMap["quad_ndi:675"] = {rtwname: "<Root>/From68"};
	this.rtwnameHashMap["<Root>/From69"] = {sid: "quad_ndi:676"};
	this.sidHashMap["quad_ndi:676"] = {rtwname: "<Root>/From69"};
	this.rtwnameHashMap["<Root>/From7"] = {sid: "quad_ndi:844"};
	this.sidHashMap["quad_ndi:844"] = {rtwname: "<Root>/From7"};
	this.rtwnameHashMap["<Root>/From70"] = {sid: "quad_ndi:677"};
	this.sidHashMap["quad_ndi:677"] = {rtwname: "<Root>/From70"};
	this.rtwnameHashMap["<Root>/From71"] = {sid: "quad_ndi:678"};
	this.sidHashMap["quad_ndi:678"] = {rtwname: "<Root>/From71"};
	this.rtwnameHashMap["<Root>/Gain1"] = {sid: "quad_ndi:815"};
	this.sidHashMap["quad_ndi:815"] = {rtwname: "<Root>/Gain1"};
	this.rtwnameHashMap["<Root>/Gain2"] = {sid: "quad_ndi:846"};
	this.sidHashMap["quad_ndi:846"] = {rtwname: "<Root>/Gain2"};
	this.rtwnameHashMap["<Root>/Goto1"] = {sid: "quad_ndi:833"};
	this.sidHashMap["quad_ndi:833"] = {rtwname: "<Root>/Goto1"};
	this.rtwnameHashMap["<Root>/Goto2"] = {sid: "quad_ndi:836"};
	this.sidHashMap["quad_ndi:836"] = {rtwname: "<Root>/Goto2"};
	this.rtwnameHashMap["<Root>/Goto20"] = {sid: "quad_ndi:401"};
	this.sidHashMap["quad_ndi:401"] = {rtwname: "<Root>/Goto20"};
	this.rtwnameHashMap["<Root>/Goto31"] = {sid: "quad_ndi:406"};
	this.sidHashMap["quad_ndi:406"] = {rtwname: "<Root>/Goto31"};
	this.rtwnameHashMap["<Root>/Goto35"] = {sid: "quad_ndi:410"};
	this.sidHashMap["quad_ndi:410"] = {rtwname: "<Root>/Goto35"};
	this.rtwnameHashMap["<Root>/Goto42"] = {sid: "quad_ndi:417"};
	this.sidHashMap["quad_ndi:417"] = {rtwname: "<Root>/Goto42"};
	this.rtwnameHashMap["<Root>/Goto43"] = {sid: "quad_ndi:418"};
	this.sidHashMap["quad_ndi:418"] = {rtwname: "<Root>/Goto43"};
	this.rtwnameHashMap["<Root>/Goto48"] = {sid: "quad_ndi:423"};
	this.sidHashMap["quad_ndi:423"] = {rtwname: "<Root>/Goto48"};
	this.rtwnameHashMap["<Root>/Goto51"] = {sid: "quad_ndi:426"};
	this.sidHashMap["quad_ndi:426"] = {rtwname: "<Root>/Goto51"};
	this.rtwnameHashMap["<Root>/MATLAB Function"] = {sid: "quad_ndi:640"};
	this.sidHashMap["quad_ndi:640"] = {rtwname: "<Root>/MATLAB Function"};
	this.rtwnameHashMap["<Root>/MATLAB Function1"] = {sid: "quad_ndi:818"};
	this.sidHashMap["quad_ndi:818"] = {rtwname: "<Root>/MATLAB Function1"};
	this.rtwnameHashMap["<Root>/Mux"] = {sid: "quad_ndi:681"};
	this.sidHashMap["quad_ndi:681"] = {rtwname: "<Root>/Mux"};
	this.rtwnameHashMap["<Root>/Mux1"] = {sid: "quad_ndi:394"};
	this.sidHashMap["quad_ndi:394"] = {rtwname: "<Root>/Mux1"};
	this.rtwnameHashMap["<Root>/NDI LAW"] = {sid: "quad_ndi:683"};
	this.sidHashMap["quad_ndi:683"] = {rtwname: "<Root>/NDI LAW"};
	this.rtwnameHashMap["<Root>/Product2"] = {sid: "quad_ndi:106"};
	this.sidHashMap["quad_ndi:106"] = {rtwname: "<Root>/Product2"};
	this.rtwnameHashMap["<Root>/Product3"] = {sid: "quad_ndi:389"};
	this.sidHashMap["quad_ndi:389"] = {rtwname: "<Root>/Product3"};
	this.rtwnameHashMap["<Root>/Product4"] = {sid: "quad_ndi:390"};
	this.sidHashMap["quad_ndi:390"] = {rtwname: "<Root>/Product4"};
	this.rtwnameHashMap["<Root>/Scope"] = {sid: "quad_ndi:661"};
	this.sidHashMap["quad_ndi:661"] = {rtwname: "<Root>/Scope"};
	this.rtwnameHashMap["<Root>/Scope1"] = {sid: "quad_ndi:814"};
	this.sidHashMap["quad_ndi:814"] = {rtwname: "<Root>/Scope1"};
	this.rtwnameHashMap["<Root>/Selector"] = {sid: "quad_ndi:841"};
	this.sidHashMap["quad_ndi:841"] = {rtwname: "<Root>/Selector"};
	this.rtwnameHashMap["<Root>/Selector1"] = {sid: "quad_ndi:391"};
	this.sidHashMap["quad_ndi:391"] = {rtwname: "<Root>/Selector1"};
	this.rtwnameHashMap["<Root>/Selector2"] = {sid: "quad_ndi:392"};
	this.sidHashMap["quad_ndi:392"] = {rtwname: "<Root>/Selector2"};
	this.rtwnameHashMap["<Root>/Selector3"] = {sid: "quad_ndi:843"};
	this.sidHashMap["quad_ndi:843"] = {rtwname: "<Root>/Selector3"};
	this.rtwnameHashMap["<Root>/Selector4"] = {sid: "quad_ndi:845"};
	this.sidHashMap["quad_ndi:845"] = {rtwname: "<Root>/Selector4"};
	this.rtwnameHashMap["<Root>/Selector7"] = {sid: "quad_ndi:107"};
	this.sidHashMap["quad_ndi:107"] = {rtwname: "<Root>/Selector7"};
	this.rtwnameHashMap["<Root>/Subsystem"] = {sid: "quad_ndi:381"};
	this.sidHashMap["quad_ndi:381"] = {rtwname: "<Root>/Subsystem"};
	this.rtwnameHashMap["<Root>/Subsystem1"] = {sid: "quad_ndi:690"};
	this.sidHashMap["quad_ndi:690"] = {rtwname: "<Root>/Subsystem1"};
	this.rtwnameHashMap["<Root>/angle_control"] = {sid: "quad_ndi:638"};
	this.sidHashMap["quad_ndi:638"] = {rtwname: "<Root>/angle_control"};
	this.rtwnameHashMap["<Root>/control_attitude"] = {sid: "quad_ndi:696"};
	this.sidHashMap["quad_ndi:696"] = {rtwname: "<Root>/control_attitude"};
	this.rtwnameHashMap["<Root>/control_attitude_rates"] = {sid: "quad_ndi:713"};
	this.sidHashMap["quad_ndi:713"] = {rtwname: "<Root>/control_attitude_rates"};
	this.rtwnameHashMap["<Root>/min_pwm"] = {sid: "quad_ndi:711"};
	this.sidHashMap["quad_ndi:711"] = {rtwname: "<Root>/min_pwm"};
	this.rtwnameHashMap["<Root>/mixer"] = {sid: "quad_ndi:639"};
	this.sidHashMap["quad_ndi:639"] = {rtwname: "<Root>/mixer"};
	this.rtwnameHashMap["<Root>/rate_control"] = {sid: "quad_ndi:634"};
	this.sidHashMap["quad_ndi:634"] = {rtwname: "<Root>/rate_control"};
	this.rtwnameHashMap["<Root>/cmd"] = {sid: "quad_ndi:831"};
	this.sidHashMap["quad_ndi:831"] = {rtwname: "<Root>/cmd"};
	this.rtwnameHashMap["<S1>:1"] = {sid: "quad_ndi:640:1"};
	this.sidHashMap["quad_ndi:640:1"] = {rtwname: "<S1>:1"};
	this.rtwnameHashMap["<S2>:1"] = {sid: "quad_ndi:818:1"};
	this.sidHashMap["quad_ndi:818:1"] = {rtwname: "<S2>:1"};
	this.rtwnameHashMap["<S3>:1"] = {sid: "quad_ndi:683:1"};
	this.sidHashMap["quad_ndi:683:1"] = {rtwname: "<S3>:1"};
	this.rtwnameHashMap["<S3>:1:4"] = {sid: "quad_ndi:683:1:4"};
	this.sidHashMap["quad_ndi:683:1:4"] = {rtwname: "<S3>:1:4"};
	this.rtwnameHashMap["<S4>/omega_ref"] = {sid: "quad_ndi:382"};
	this.sidHashMap["quad_ndi:382"] = {rtwname: "<S4>/omega_ref"};
	this.rtwnameHashMap["<S4>/yawrate_ref"] = {sid: "quad_ndi:383"};
	this.sidHashMap["quad_ndi:383"] = {rtwname: "<S4>/yawrate_ref"};
	this.rtwnameHashMap["<S4>/Mux1"] = {sid: "quad_ndi:380"};
	this.sidHashMap["quad_ndi:380"] = {rtwname: "<S4>/Mux1"};
	this.rtwnameHashMap["<S4>/Selector"] = {sid: "quad_ndi:379"};
	this.sidHashMap["quad_ndi:379"] = {rtwname: "<S4>/Selector"};
	this.rtwnameHashMap["<S4>/rate_ref"] = {sid: "quad_ndi:384"};
	this.sidHashMap["quad_ndi:384"] = {rtwname: "<S4>/rate_ref"};
	this.rtwnameHashMap["<S5>/rates_sp"] = {sid: "quad_ndi:691"};
	this.sidHashMap["quad_ndi:691"] = {rtwname: "<S5>/rates_sp"};
	this.rtwnameHashMap["<S5>/yawrate_sp"] = {sid: "quad_ndi:692"};
	this.sidHashMap["quad_ndi:692"] = {rtwname: "<S5>/yawrate_sp"};
	this.rtwnameHashMap["<S5>/Mux1"] = {sid: "quad_ndi:693"};
	this.sidHashMap["quad_ndi:693"] = {rtwname: "<S5>/Mux1"};
	this.rtwnameHashMap["<S5>/Selector"] = {sid: "quad_ndi:694"};
	this.sidHashMap["quad_ndi:694"] = {rtwname: "<S5>/Selector"};
	this.rtwnameHashMap["<S5>/rates_setpoint"] = {sid: "quad_ndi:695"};
	this.sidHashMap["quad_ndi:695"] = {rtwname: "<S5>/rates_setpoint"};
	this.rtwnameHashMap["<S6>:1"] = {sid: "quad_ndi:638:1"};
	this.sidHashMap["quad_ndi:638:1"] = {rtwname: "<S6>:1"};
	this.rtwnameHashMap["<S7>/attitude_setpoint"] = {sid: "quad_ndi:697"};
	this.sidHashMap["quad_ndi:697"] = {rtwname: "<S7>/attitude_setpoint"};
	this.rtwnameHashMap["<S7>/vehicle_attitude"] = {sid: "quad_ndi:698"};
	this.sidHashMap["quad_ndi:698"] = {rtwname: "<S7>/vehicle_attitude"};
	this.rtwnameHashMap["<S7>/attitude_param_Kp"] = {sid: "quad_ndi:699"};
	this.sidHashMap["quad_ndi:699"] = {rtwname: "<S7>/attitude_param_Kp"};
	this.rtwnameHashMap["<S7>/Demux"] = {sid: "quad_ndi:700"};
	this.sidHashMap["quad_ndi:700"] = {rtwname: "<S7>/Demux"};
	this.rtwnameHashMap["<S7>/Demux1"] = {sid: "quad_ndi:701"};
	this.sidHashMap["quad_ndi:701"] = {rtwname: "<S7>/Demux1"};
	this.rtwnameHashMap["<S7>/Demux2"] = {sid: "quad_ndi:702"};
	this.sidHashMap["quad_ndi:702"] = {rtwname: "<S7>/Demux2"};
	this.rtwnameHashMap["<S7>/Mux"] = {sid: "quad_ndi:703"};
	this.sidHashMap["quad_ndi:703"] = {rtwname: "<S7>/Mux"};
	this.rtwnameHashMap["<S7>/PID Controller1"] = {sid: "quad_ndi:704"};
	this.sidHashMap["quad_ndi:704"] = {rtwname: "<S7>/PID Controller1"};
	this.rtwnameHashMap["<S7>/PID Controller2"] = {sid: "quad_ndi:705"};
	this.sidHashMap["quad_ndi:705"] = {rtwname: "<S7>/PID Controller2"};
	this.rtwnameHashMap["<S7>/PID Controller3"] = {sid: "quad_ndi:706"};
	this.sidHashMap["quad_ndi:706"] = {rtwname: "<S7>/PID Controller3"};
	this.rtwnameHashMap["<S7>/Sum1"] = {sid: "quad_ndi:707"};
	this.sidHashMap["quad_ndi:707"] = {rtwname: "<S7>/Sum1"};
	this.rtwnameHashMap["<S7>/Sum5"] = {sid: "quad_ndi:708"};
	this.sidHashMap["quad_ndi:708"] = {rtwname: "<S7>/Sum5"};
	this.rtwnameHashMap["<S7>/Sum6"] = {sid: "quad_ndi:709"};
	this.sidHashMap["quad_ndi:709"] = {rtwname: "<S7>/Sum6"};
	this.rtwnameHashMap["<S7>/euler_rates_2_body_rates"] = {sid: "quad_ndi:830"};
	this.sidHashMap["quad_ndi:830"] = {rtwname: "<S7>/euler_rates_2_body_rates"};
	this.rtwnameHashMap["<S7>/rates_setpoint"] = {sid: "quad_ndi:710"};
	this.sidHashMap["quad_ndi:710"] = {rtwname: "<S7>/rates_setpoint"};
	this.rtwnameHashMap["<S8>/rates_sp"] = {sid: "quad_ndi:714"};
	this.sidHashMap["quad_ndi:714"] = {rtwname: "<S8>/rates_sp"};
	this.rtwnameHashMap["<S8>/rates"] = {sid: "quad_ndi:715"};
	this.sidHashMap["quad_ndi:715"] = {rtwname: "<S8>/rates"};
	this.rtwnameHashMap["<S8>/param_rate_kp"] = {sid: "quad_ndi:716"};
	this.sidHashMap["quad_ndi:716"] = {rtwname: "<S8>/param_rate_kp"};
	this.rtwnameHashMap["<S8>/param_rate_ki"] = {sid: "quad_ndi:717"};
	this.sidHashMap["quad_ndi:717"] = {rtwname: "<S8>/param_rate_ki"};
	this.rtwnameHashMap["<S8>/param_rate_kd"] = {sid: "quad_ndi:718"};
	this.sidHashMap["quad_ndi:718"] = {rtwname: "<S8>/param_rate_kd"};
	this.rtwnameHashMap["<S8>/param_rate_N"] = {sid: "quad_ndi:719"};
	this.sidHashMap["quad_ndi:719"] = {rtwname: "<S8>/param_rate_N"};
	this.rtwnameHashMap["<S8>/Discrete_PID_measurement_D"] = {sid: "quad_ndi:720"};
	this.sidHashMap["quad_ndi:720"] = {rtwname: "<S8>/Discrete_PID_measurement_D"};
	this.rtwnameHashMap["<S8>/Discrete_PID_measurement_D1"] = {sid: "quad_ndi:737"};
	this.sidHashMap["quad_ndi:737"] = {rtwname: "<S8>/Discrete_PID_measurement_D1"};
	this.rtwnameHashMap["<S8>/Discrete_PID_measurement_D2"] = {sid: "quad_ndi:754"};
	this.sidHashMap["quad_ndi:754"] = {rtwname: "<S8>/Discrete_PID_measurement_D2"};
	this.rtwnameHashMap["<S8>/Mux1"] = {sid: "quad_ndi:771"};
	this.sidHashMap["quad_ndi:771"] = {rtwname: "<S8>/Mux1"};
	this.rtwnameHashMap["<S8>/Selector"] = {sid: "quad_ndi:772"};
	this.sidHashMap["quad_ndi:772"] = {rtwname: "<S8>/Selector"};
	this.rtwnameHashMap["<S8>/Selector1"] = {sid: "quad_ndi:773"};
	this.sidHashMap["quad_ndi:773"] = {rtwname: "<S8>/Selector1"};
	this.rtwnameHashMap["<S8>/Selector10"] = {sid: "quad_ndi:774"};
	this.sidHashMap["quad_ndi:774"] = {rtwname: "<S8>/Selector10"};
	this.rtwnameHashMap["<S8>/Selector11"] = {sid: "quad_ndi:775"};
	this.sidHashMap["quad_ndi:775"] = {rtwname: "<S8>/Selector11"};
	this.rtwnameHashMap["<S8>/Selector12"] = {sid: "quad_ndi:776"};
	this.sidHashMap["quad_ndi:776"] = {rtwname: "<S8>/Selector12"};
	this.rtwnameHashMap["<S8>/Selector13"] = {sid: "quad_ndi:777"};
	this.sidHashMap["quad_ndi:777"] = {rtwname: "<S8>/Selector13"};
	this.rtwnameHashMap["<S8>/Selector14"] = {sid: "quad_ndi:778"};
	this.sidHashMap["quad_ndi:778"] = {rtwname: "<S8>/Selector14"};
	this.rtwnameHashMap["<S8>/Selector15"] = {sid: "quad_ndi:779"};
	this.sidHashMap["quad_ndi:779"] = {rtwname: "<S8>/Selector15"};
	this.rtwnameHashMap["<S8>/Selector16"] = {sid: "quad_ndi:780"};
	this.sidHashMap["quad_ndi:780"] = {rtwname: "<S8>/Selector16"};
	this.rtwnameHashMap["<S8>/Selector17"] = {sid: "quad_ndi:781"};
	this.sidHashMap["quad_ndi:781"] = {rtwname: "<S8>/Selector17"};
	this.rtwnameHashMap["<S8>/Selector2"] = {sid: "quad_ndi:782"};
	this.sidHashMap["quad_ndi:782"] = {rtwname: "<S8>/Selector2"};
	this.rtwnameHashMap["<S8>/Selector3"] = {sid: "quad_ndi:783"};
	this.sidHashMap["quad_ndi:783"] = {rtwname: "<S8>/Selector3"};
	this.rtwnameHashMap["<S8>/Selector4"] = {sid: "quad_ndi:784"};
	this.sidHashMap["quad_ndi:784"] = {rtwname: "<S8>/Selector4"};
	this.rtwnameHashMap["<S8>/Selector5"] = {sid: "quad_ndi:785"};
	this.sidHashMap["quad_ndi:785"] = {rtwname: "<S8>/Selector5"};
	this.rtwnameHashMap["<S8>/Selector6"] = {sid: "quad_ndi:786"};
	this.sidHashMap["quad_ndi:786"] = {rtwname: "<S8>/Selector6"};
	this.rtwnameHashMap["<S8>/Selector7"] = {sid: "quad_ndi:787"};
	this.sidHashMap["quad_ndi:787"] = {rtwname: "<S8>/Selector7"};
	this.rtwnameHashMap["<S8>/Selector8"] = {sid: "quad_ndi:788"};
	this.sidHashMap["quad_ndi:788"] = {rtwname: "<S8>/Selector8"};
	this.rtwnameHashMap["<S8>/Selector9"] = {sid: "quad_ndi:789"};
	this.sidHashMap["quad_ndi:789"] = {rtwname: "<S8>/Selector9"};
	this.rtwnameHashMap["<S8>/omega_dot_ref"] = {sid: "quad_ndi:790"};
	this.sidHashMap["quad_ndi:790"] = {rtwname: "<S8>/omega_dot_ref"};
	this.rtwnameHashMap["<S9>:1"] = {sid: "quad_ndi:639:1"};
	this.sidHashMap["quad_ndi:639:1"] = {rtwname: "<S9>:1"};
	this.rtwnameHashMap["<S10>:1"] = {sid: "quad_ndi:634:1"};
	this.sidHashMap["quad_ndi:634:1"] = {rtwname: "<S10>:1"};
	this.rtwnameHashMap["<S11>/u"] = {sid: "quad_ndi:704:1"};
	this.sidHashMap["quad_ndi:704:1"] = {rtwname: "<S11>/u"};
	this.rtwnameHashMap["<S11>/P"] = {sid: "quad_ndi:704:1655"};
	this.sidHashMap["quad_ndi:704:1655"] = {rtwname: "<S11>/P"};
	this.rtwnameHashMap["<S11>/POut"] = {sid: "quad_ndi:704:1656"};
	this.sidHashMap["quad_ndi:704:1656"] = {rtwname: "<S11>/POut"};
	this.rtwnameHashMap["<S11>/y"] = {sid: "quad_ndi:704:10"};
	this.sidHashMap["quad_ndi:704:10"] = {rtwname: "<S11>/y"};
	this.rtwnameHashMap["<S12>/u"] = {sid: "quad_ndi:705:1"};
	this.sidHashMap["quad_ndi:705:1"] = {rtwname: "<S12>/u"};
	this.rtwnameHashMap["<S12>/P"] = {sid: "quad_ndi:705:1655"};
	this.sidHashMap["quad_ndi:705:1655"] = {rtwname: "<S12>/P"};
	this.rtwnameHashMap["<S12>/POut"] = {sid: "quad_ndi:705:1656"};
	this.sidHashMap["quad_ndi:705:1656"] = {rtwname: "<S12>/POut"};
	this.rtwnameHashMap["<S12>/y"] = {sid: "quad_ndi:705:10"};
	this.sidHashMap["quad_ndi:705:10"] = {rtwname: "<S12>/y"};
	this.rtwnameHashMap["<S13>/u"] = {sid: "quad_ndi:706:1"};
	this.sidHashMap["quad_ndi:706:1"] = {rtwname: "<S13>/u"};
	this.rtwnameHashMap["<S13>/P"] = {sid: "quad_ndi:706:1655"};
	this.sidHashMap["quad_ndi:706:1655"] = {rtwname: "<S13>/P"};
	this.rtwnameHashMap["<S13>/POut"] = {sid: "quad_ndi:706:1656"};
	this.sidHashMap["quad_ndi:706:1656"] = {rtwname: "<S13>/POut"};
	this.rtwnameHashMap["<S13>/y"] = {sid: "quad_ndi:706:10"};
	this.sidHashMap["quad_ndi:706:10"] = {rtwname: "<S13>/y"};
	this.rtwnameHashMap["<S14>:1"] = {sid: "quad_ndi:830:1"};
	this.sidHashMap["quad_ndi:830:1"] = {rtwname: "<S14>:1"};
	this.rtwnameHashMap["<S15>/Ref"] = {sid: "quad_ndi:721"};
	this.sidHashMap["quad_ndi:721"] = {rtwname: "<S15>/Ref"};
	this.rtwnameHashMap["<S15>/Meas"] = {sid: "quad_ndi:722"};
	this.sidHashMap["quad_ndi:722"] = {rtwname: "<S15>/Meas"};
	this.rtwnameHashMap["<S15>/P"] = {sid: "quad_ndi:723"};
	this.sidHashMap["quad_ndi:723"] = {rtwname: "<S15>/P"};
	this.rtwnameHashMap["<S15>/I"] = {sid: "quad_ndi:724"};
	this.sidHashMap["quad_ndi:724"] = {rtwname: "<S15>/I"};
	this.rtwnameHashMap["<S15>/D"] = {sid: "quad_ndi:725"};
	this.sidHashMap["quad_ndi:725"] = {rtwname: "<S15>/D"};
	this.rtwnameHashMap["<S15>/N"] = {sid: "quad_ndi:726"};
	this.sidHashMap["quad_ndi:726"] = {rtwname: "<S15>/N"};
	this.rtwnameHashMap["<S15>/Dout"] = {sid: "quad_ndi:727"};
	this.sidHashMap["quad_ndi:727"] = {rtwname: "<S15>/Dout"};
	this.rtwnameHashMap["<S15>/Filter State"] = {sid: "quad_ndi:728"};
	this.sidHashMap["quad_ndi:728"] = {rtwname: "<S15>/Filter State"};
	this.rtwnameHashMap["<S15>/Integrator"] = {sid: "quad_ndi:729"};
	this.sidHashMap["quad_ndi:729"] = {rtwname: "<S15>/Integrator"};
	this.rtwnameHashMap["<S15>/Iout"] = {sid: "quad_ndi:730"};
	this.sidHashMap["quad_ndi:730"] = {rtwname: "<S15>/Iout"};
	this.rtwnameHashMap["<S15>/NOut"] = {sid: "quad_ndi:731"};
	this.sidHashMap["quad_ndi:731"] = {rtwname: "<S15>/NOut"};
	this.rtwnameHashMap["<S15>/Pout"] = {sid: "quad_ndi:732"};
	this.sidHashMap["quad_ndi:732"] = {rtwname: "<S15>/Pout"};
	this.rtwnameHashMap["<S15>/Sum1"] = {sid: "quad_ndi:733"};
	this.sidHashMap["quad_ndi:733"] = {rtwname: "<S15>/Sum1"};
	this.rtwnameHashMap["<S15>/Sum2"] = {sid: "quad_ndi:734"};
	this.sidHashMap["quad_ndi:734"] = {rtwname: "<S15>/Sum2"};
	this.rtwnameHashMap["<S15>/error"] = {sid: "quad_ndi:735"};
	this.sidHashMap["quad_ndi:735"] = {rtwname: "<S15>/error"};
	this.rtwnameHashMap["<S15>/y"] = {sid: "quad_ndi:736"};
	this.sidHashMap["quad_ndi:736"] = {rtwname: "<S15>/y"};
	this.rtwnameHashMap["<S16>/Ref"] = {sid: "quad_ndi:738"};
	this.sidHashMap["quad_ndi:738"] = {rtwname: "<S16>/Ref"};
	this.rtwnameHashMap["<S16>/Meas"] = {sid: "quad_ndi:739"};
	this.sidHashMap["quad_ndi:739"] = {rtwname: "<S16>/Meas"};
	this.rtwnameHashMap["<S16>/P"] = {sid: "quad_ndi:740"};
	this.sidHashMap["quad_ndi:740"] = {rtwname: "<S16>/P"};
	this.rtwnameHashMap["<S16>/I"] = {sid: "quad_ndi:741"};
	this.sidHashMap["quad_ndi:741"] = {rtwname: "<S16>/I"};
	this.rtwnameHashMap["<S16>/D"] = {sid: "quad_ndi:742"};
	this.sidHashMap["quad_ndi:742"] = {rtwname: "<S16>/D"};
	this.rtwnameHashMap["<S16>/N"] = {sid: "quad_ndi:743"};
	this.sidHashMap["quad_ndi:743"] = {rtwname: "<S16>/N"};
	this.rtwnameHashMap["<S16>/Dout"] = {sid: "quad_ndi:744"};
	this.sidHashMap["quad_ndi:744"] = {rtwname: "<S16>/Dout"};
	this.rtwnameHashMap["<S16>/Filter State"] = {sid: "quad_ndi:745"};
	this.sidHashMap["quad_ndi:745"] = {rtwname: "<S16>/Filter State"};
	this.rtwnameHashMap["<S16>/Integrator"] = {sid: "quad_ndi:746"};
	this.sidHashMap["quad_ndi:746"] = {rtwname: "<S16>/Integrator"};
	this.rtwnameHashMap["<S16>/Iout"] = {sid: "quad_ndi:747"};
	this.sidHashMap["quad_ndi:747"] = {rtwname: "<S16>/Iout"};
	this.rtwnameHashMap["<S16>/NOut"] = {sid: "quad_ndi:748"};
	this.sidHashMap["quad_ndi:748"] = {rtwname: "<S16>/NOut"};
	this.rtwnameHashMap["<S16>/Pout"] = {sid: "quad_ndi:749"};
	this.sidHashMap["quad_ndi:749"] = {rtwname: "<S16>/Pout"};
	this.rtwnameHashMap["<S16>/Sum1"] = {sid: "quad_ndi:750"};
	this.sidHashMap["quad_ndi:750"] = {rtwname: "<S16>/Sum1"};
	this.rtwnameHashMap["<S16>/Sum2"] = {sid: "quad_ndi:751"};
	this.sidHashMap["quad_ndi:751"] = {rtwname: "<S16>/Sum2"};
	this.rtwnameHashMap["<S16>/error"] = {sid: "quad_ndi:752"};
	this.sidHashMap["quad_ndi:752"] = {rtwname: "<S16>/error"};
	this.rtwnameHashMap["<S16>/y"] = {sid: "quad_ndi:753"};
	this.sidHashMap["quad_ndi:753"] = {rtwname: "<S16>/y"};
	this.rtwnameHashMap["<S17>/Ref"] = {sid: "quad_ndi:755"};
	this.sidHashMap["quad_ndi:755"] = {rtwname: "<S17>/Ref"};
	this.rtwnameHashMap["<S17>/Meas"] = {sid: "quad_ndi:756"};
	this.sidHashMap["quad_ndi:756"] = {rtwname: "<S17>/Meas"};
	this.rtwnameHashMap["<S17>/P"] = {sid: "quad_ndi:757"};
	this.sidHashMap["quad_ndi:757"] = {rtwname: "<S17>/P"};
	this.rtwnameHashMap["<S17>/I"] = {sid: "quad_ndi:758"};
	this.sidHashMap["quad_ndi:758"] = {rtwname: "<S17>/I"};
	this.rtwnameHashMap["<S17>/D"] = {sid: "quad_ndi:759"};
	this.sidHashMap["quad_ndi:759"] = {rtwname: "<S17>/D"};
	this.rtwnameHashMap["<S17>/N"] = {sid: "quad_ndi:760"};
	this.sidHashMap["quad_ndi:760"] = {rtwname: "<S17>/N"};
	this.rtwnameHashMap["<S17>/Dout"] = {sid: "quad_ndi:761"};
	this.sidHashMap["quad_ndi:761"] = {rtwname: "<S17>/Dout"};
	this.rtwnameHashMap["<S17>/Filter State"] = {sid: "quad_ndi:762"};
	this.sidHashMap["quad_ndi:762"] = {rtwname: "<S17>/Filter State"};
	this.rtwnameHashMap["<S17>/Integrator"] = {sid: "quad_ndi:763"};
	this.sidHashMap["quad_ndi:763"] = {rtwname: "<S17>/Integrator"};
	this.rtwnameHashMap["<S17>/Iout"] = {sid: "quad_ndi:764"};
	this.sidHashMap["quad_ndi:764"] = {rtwname: "<S17>/Iout"};
	this.rtwnameHashMap["<S17>/NOut"] = {sid: "quad_ndi:765"};
	this.sidHashMap["quad_ndi:765"] = {rtwname: "<S17>/NOut"};
	this.rtwnameHashMap["<S17>/Pout"] = {sid: "quad_ndi:766"};
	this.sidHashMap["quad_ndi:766"] = {rtwname: "<S17>/Pout"};
	this.rtwnameHashMap["<S17>/Sum1"] = {sid: "quad_ndi:767"};
	this.sidHashMap["quad_ndi:767"] = {rtwname: "<S17>/Sum1"};
	this.rtwnameHashMap["<S17>/Sum2"] = {sid: "quad_ndi:768"};
	this.sidHashMap["quad_ndi:768"] = {rtwname: "<S17>/Sum2"};
	this.rtwnameHashMap["<S17>/error"] = {sid: "quad_ndi:769"};
	this.sidHashMap["quad_ndi:769"] = {rtwname: "<S17>/error"};
	this.rtwnameHashMap["<S17>/y"] = {sid: "quad_ndi:770"};
	this.sidHashMap["quad_ndi:770"] = {rtwname: "<S17>/y"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();
