function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "INS"};
	this.sidHashMap["INS"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<Root>/IMU1"] = {sid: "INS:4"};
	this.sidHashMap["INS:4"] = {rtwname: "<Root>/IMU1"};
	this.rtwnameHashMap["<Root>/Mag"] = {sid: "INS:5"};
	this.sidHashMap["INS:5"] = {rtwname: "<Root>/Mag"};
	this.rtwnameHashMap["<Root>/GPS_uBlox"] = {sid: "INS:6"};
	this.sidHashMap["INS:6"] = {rtwname: "<Root>/GPS_uBlox"};
	this.rtwnameHashMap["<Root>/Baro"] = {sid: "INS:7"};
	this.sidHashMap["INS:7"] = {rtwname: "<Root>/Baro"};
	this.rtwnameHashMap["<Root>/Sensor_Param"] = {sid: "INS:8"};
	this.sidHashMap["INS:8"] = {rtwname: "<Root>/Sensor_Param"};
	this.rtwnameHashMap["<Root>/CF_INS"] = {sid: "INS:2"};
	this.sidHashMap["INS:2"] = {rtwname: "<Root>/CF_INS"};
	this.rtwnameHashMap["<Root>/Sensor_Processing"] = {sid: "INS:1"};
	this.sidHashMap["INS:1"] = {rtwname: "<Root>/Sensor_Processing"};
	this.rtwnameHashMap["<Root>/INS_Out"] = {sid: "INS:9"};
	this.sidHashMap["INS:9"] = {rtwname: "<Root>/INS_Out"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();
