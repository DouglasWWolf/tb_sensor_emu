# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "LVDS_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "PATTERN_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "SYNC_PULSE_LENGTH" -parent ${Page_0}


}

proc update_PARAM_VALUE.LVDS_WIDTH { PARAM_VALUE.LVDS_WIDTH } {
	# Procedure called to update LVDS_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.LVDS_WIDTH { PARAM_VALUE.LVDS_WIDTH } {
	# Procedure called to validate LVDS_WIDTH
	return true
}

proc update_PARAM_VALUE.PATTERN_WIDTH { PARAM_VALUE.PATTERN_WIDTH } {
	# Procedure called to update PATTERN_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.PATTERN_WIDTH { PARAM_VALUE.PATTERN_WIDTH } {
	# Procedure called to validate PATTERN_WIDTH
	return true
}

proc update_PARAM_VALUE.SYNC_PULSE_LENGTH { PARAM_VALUE.SYNC_PULSE_LENGTH } {
	# Procedure called to update SYNC_PULSE_LENGTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.SYNC_PULSE_LENGTH { PARAM_VALUE.SYNC_PULSE_LENGTH } {
	# Procedure called to validate SYNC_PULSE_LENGTH
	return true
}


proc update_MODELPARAM_VALUE.PATTERN_WIDTH { MODELPARAM_VALUE.PATTERN_WIDTH PARAM_VALUE.PATTERN_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.PATTERN_WIDTH}] ${MODELPARAM_VALUE.PATTERN_WIDTH}
}

proc update_MODELPARAM_VALUE.LVDS_WIDTH { MODELPARAM_VALUE.LVDS_WIDTH PARAM_VALUE.LVDS_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.LVDS_WIDTH}] ${MODELPARAM_VALUE.LVDS_WIDTH}
}

proc update_MODELPARAM_VALUE.SYNC_PULSE_LENGTH { MODELPARAM_VALUE.SYNC_PULSE_LENGTH PARAM_VALUE.SYNC_PULSE_LENGTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.SYNC_PULSE_LENGTH}] ${MODELPARAM_VALUE.SYNC_PULSE_LENGTH}
}

