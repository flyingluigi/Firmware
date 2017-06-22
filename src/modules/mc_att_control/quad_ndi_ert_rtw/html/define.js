function CodeDefine() { 
this.def = new Array();
this.def["rtObj"] = {file: "ert_main_cpp.html",line:22,type:"var"};
this.def["arg_vehicle_attitude"] = {file: "ert_main_cpp.html",line:25,type:"var"};
this.def["arg_vehicle_attitude_setpoint"] = {file: "ert_main_cpp.html",line:28,type:"var"};
this.def["arg_param"] = {file: "ert_main_cpp.html",line:31,type:"var"};
this.def["arg_cmd"] = {file: "ert_main_cpp.html",line:35,type:"var"};
this.def["rt_OneStep"] = {file: "ert_main_cpp.html",line:49,type:"fcn"};
this.def["main"] = {file: "ert_main_cpp.html",line:87,type:"fcn"};
this.def["step"] = {file: "quad_ndi_cpp.html",line:20,type:"fcn"};
this.def["initialize"] = {file: "quad_ndi_cpp.html",line:202,type:"fcn"};
this.def["getRTM"] = {file: "quad_ndi_cpp.html",line:219,type:"fcn"};
this.def["RT_MODEL"] = {file: "quad_ndi_h.html",line:34,type:"type"};
this.def["DW"] = {file: "quad_ndi_h.html",line:44,type:"type"};
this.def["ConstP"] = {file: "quad_ndi_h.html",line:52,type:"type"};
this.def["public"] = {file: "quad_ndi_h.html",line:76,type:"fcn"};
this.def["rtConstP"] = {file: "quad_ndi_data_cpp.html",line:20,type:"var"};
this.def["int8_T"] = {file: "rtwtypes_h.html",line:49,type:"type"};
this.def["uint8_T"] = {file: "rtwtypes_h.html",line:50,type:"type"};
this.def["int16_T"] = {file: "rtwtypes_h.html",line:51,type:"type"};
this.def["uint16_T"] = {file: "rtwtypes_h.html",line:52,type:"type"};
this.def["int32_T"] = {file: "rtwtypes_h.html",line:53,type:"type"};
this.def["uint32_T"] = {file: "rtwtypes_h.html",line:54,type:"type"};
this.def["real32_T"] = {file: "rtwtypes_h.html",line:55,type:"type"};
this.def["real64_T"] = {file: "rtwtypes_h.html",line:56,type:"type"};
this.def["real_T"] = {file: "rtwtypes_h.html",line:62,type:"type"};
this.def["time_T"] = {file: "rtwtypes_h.html",line:63,type:"type"};
this.def["boolean_T"] = {file: "rtwtypes_h.html",line:64,type:"type"};
this.def["int_T"] = {file: "rtwtypes_h.html",line:65,type:"type"};
this.def["uint_T"] = {file: "rtwtypes_h.html",line:66,type:"type"};
this.def["ulong_T"] = {file: "rtwtypes_h.html",line:67,type:"type"};
this.def["char_T"] = {file: "rtwtypes_h.html",line:68,type:"type"};
this.def["uchar_T"] = {file: "rtwtypes_h.html",line:69,type:"type"};
this.def["byte_T"] = {file: "rtwtypes_h.html",line:70,type:"type"};
this.def["pointer_T"] = {file: "rtwtypes_h.html",line:88,type:"type"};
}
CodeDefine.instance = new CodeDefine();
var testHarnessInfo = {OwnerFileName: "", HarnessOwner: "", HarnessName: "", IsTestHarness: "0"};
var relPathToBuildDir = "../ert_main.c";
var fileSep = "\\";
var isPC = true;
function Html2SrcLink() {
	this.html2SrcPath = new Array;
	this.html2Root = new Array;
	this.html2SrcPath["ert_main_cpp.html"] = "../ert_main.cpp";
	this.html2Root["ert_main_cpp.html"] = "ert_main_cpp.html";
	this.html2SrcPath["quad_ndi_cpp.html"] = "../quad_ndi.cpp";
	this.html2Root["quad_ndi_cpp.html"] = "quad_ndi_cpp.html";
	this.html2SrcPath["quad_ndi_h.html"] = "../quad_ndi.h";
	this.html2Root["quad_ndi_h.html"] = "quad_ndi_h.html";
	this.html2SrcPath["quad_ndi_data_cpp.html"] = "../quad_ndi_data.cpp";
	this.html2Root["quad_ndi_data_cpp.html"] = "quad_ndi_data_cpp.html";
	this.html2SrcPath["rtwtypes_h.html"] = "../rtwtypes.h";
	this.html2Root["rtwtypes_h.html"] = "rtwtypes_h.html";
	this.getLink2Src = function (htmlFileName) {
		 if (this.html2SrcPath[htmlFileName])
			 return this.html2SrcPath[htmlFileName];
		 else
			 return null;
	}
	this.getLinkFromRoot = function (htmlFileName) {
		 if (this.html2Root[htmlFileName])
			 return this.html2Root[htmlFileName];
		 else
			 return null;
	}
}
Html2SrcLink.instance = new Html2SrcLink();
var fileList = [
"ert_main_cpp.html","quad_ndi_cpp.html","quad_ndi_h.html","quad_ndi_data_cpp.html","rtwtypes_h.html"];
