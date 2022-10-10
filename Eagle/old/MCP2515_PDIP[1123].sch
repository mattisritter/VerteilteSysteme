<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="7.7.0">
<drawing>
<settings>
<setting alwaysvectorfont="yes"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="50" name="dxf" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="53" name="tGND_GNDA" color="7" fill="1" visible="no" active="no"/>
<layer number="54" name="bGND_GNDA" color="7" fill="1" visible="no" active="no"/>
<layer number="55" name="__tmp" color="7" fill="1" visible="no" active="no"/>
<layer number="56" name="wert" color="7" fill="1" visible="no" active="no"/>
<layer number="60" name="Board" color="7" fill="1" visible="no" active="no"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
<layer number="100" name="Muster" color="7" fill="1" visible="yes" active="yes"/>
<layer number="101" name="Hidden" color="15" fill="1" visible="yes" active="yes"/>
<layer number="102" name="Changes" color="12" fill="1" visible="yes" active="yes"/>
<layer number="116" name="centerDrill" color="7" fill="1" visible="yes" active="yes"/>
<layer number="200" name="200bmp" color="7" fill="1" visible="yes" active="yes"/>
<layer number="250" name="Descript" color="3" fill="1" visible="no" active="no"/>
<layer number="251" name="SMDround" color="12" fill="11" visible="no" active="no"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="sp2">
<packages>
<package name="SO-18DW">
<description>&lt;b&gt;Small Outline Integrated Circuit&lt;/b&gt; wide</description>
<wire x1="5.825" y1="-3.7" x2="-5.825" y2="-3.7" width="0.2032" layer="51"/>
<wire x1="-5.825" y1="-3.7" x2="-5.825" y2="-3.2" width="0.2032" layer="51"/>
<wire x1="-5.825" y1="-3.2" x2="-5.825" y2="3.7" width="0.2032" layer="51"/>
<wire x1="-5.825" y1="3.7" x2="5.825" y2="3.7" width="0.2032" layer="51"/>
<wire x1="5.825" y1="-3.2" x2="-5.825" y2="-3.2" width="0.2032" layer="51"/>
<wire x1="5.825" y1="3.7" x2="5.825" y2="-3.2" width="0.2032" layer="51"/>
<wire x1="5.825" y1="-3.2" x2="5.825" y2="-3.7" width="0.2032" layer="51"/>
<wire x1="-6.172" y1="5.829" x2="-6.172" y2="-5.829" width="0.2" layer="39"/>
<wire x1="-6.172" y1="-5.829" x2="6.172" y2="-5.829" width="0.2" layer="39"/>
<wire x1="6.172" y1="-5.829" x2="6.1722" y2="5.8293" width="0.2" layer="39"/>
<wire x1="6.1722" y1="5.8293" x2="-6.172" y2="5.829" width="0.2" layer="39"/>
<smd name="2" x="-3.81" y="-4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="13" x="1.27" y="4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="1" x="-5.08" y="-4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="3" x="-2.54" y="-4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="4" x="-1.27" y="-4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="14" x="0" y="4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="12" x="2.54" y="4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="11" x="3.81" y="4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="6" x="1.27" y="-4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="9" x="5.08" y="-4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="5" x="0" y="-4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="7" x="2.54" y="-4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="10" x="5.08" y="4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="8" x="3.81" y="-4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="15" x="-1.27" y="4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="16" x="-2.54" y="4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="17" x="-3.81" y="4.6" dx="0.6" dy="2.2" layer="1"/>
<smd name="18" x="-5.08" y="4.6" dx="0.6" dy="2.2" layer="1"/>
<text x="-5.08" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-5.08" y="-2.54" size="1.27" layer="27">&gt;VALUE</text>
<rectangle x1="-5.325" y1="-5.32" x2="-4.835" y2="-3.8" layer="51"/>
<rectangle x1="-4.055" y1="-5.32" x2="-3.565" y2="-3.8" layer="51"/>
<rectangle x1="-2.785" y1="-5.32" x2="-2.295" y2="-3.8" layer="51"/>
<rectangle x1="-1.515" y1="-5.32" x2="-1.025" y2="-3.8" layer="51"/>
<rectangle x1="-0.245" y1="-5.32" x2="0.245" y2="-3.8" layer="51"/>
<rectangle x1="1.025" y1="-5.32" x2="1.515" y2="-3.8" layer="51"/>
<rectangle x1="2.295" y1="-5.32" x2="2.785" y2="-3.8" layer="51"/>
<rectangle x1="3.565" y1="-5.32" x2="4.055" y2="-3.8" layer="51"/>
<rectangle x1="4.835" y1="-5.32" x2="5.325" y2="-3.8" layer="51"/>
<rectangle x1="4.835" y1="3.8" x2="5.325" y2="5.32" layer="51"/>
<rectangle x1="3.565" y1="3.8" x2="4.055" y2="5.32" layer="51"/>
<rectangle x1="2.295" y1="3.8" x2="2.785" y2="5.32" layer="51"/>
<rectangle x1="1.025" y1="3.8" x2="1.515" y2="5.32" layer="51"/>
<rectangle x1="-0.245" y1="3.8" x2="0.245" y2="5.32" layer="51"/>
<rectangle x1="-1.515" y1="3.8" x2="-1.025" y2="5.32" layer="51"/>
<rectangle x1="-2.785" y1="3.8" x2="-2.295" y2="5.32" layer="51"/>
<rectangle x1="-4.055" y1="3.8" x2="-3.565" y2="5.32" layer="51"/>
<rectangle x1="-5.325" y1="3.8" x2="-4.835" y2="5.32" layer="51"/>
</package>
<package name="DIL18">
<description>&lt;b&gt;Dual In Line Package&lt;/b&gt;</description>
<wire x1="11.43" y1="2.921" x2="-11.43" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-11.43" y1="-2.921" x2="11.43" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="11.43" y1="2.921" x2="11.43" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="-11.43" y1="2.921" x2="-11.43" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-11.43" y1="-2.921" x2="-11.43" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-11.43" y1="1.016" x2="-11.43" y2="-1.016" width="0.1524" layer="21" curve="-180"/>
<pad name="1" x="-10.16" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-7.62" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="5.08" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="8" x="7.62" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-5.08" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="-2.54" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="2.54" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="0" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="9" x="10.16" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="10" x="10.16" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="11" x="7.62" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="12" x="5.08" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="13" x="2.54" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="14" x="0" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="15" x="-2.54" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="16" x="-5.08" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="17" x="-7.62" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="18" x="-10.16" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<text x="-11.684" y="-3.048" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-9.525" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
</packages>
<symbols>
<symbol name="MCP2515">
<wire x1="-10.16" y1="17.78" x2="12.7" y2="17.78" width="0.254" layer="94"/>
<wire x1="12.7" y1="17.78" x2="12.7" y2="-17.78" width="0.254" layer="94"/>
<wire x1="12.7" y1="-17.78" x2="-10.16" y2="-17.78" width="0.254" layer="94"/>
<wire x1="-10.16" y1="-17.78" x2="-10.16" y2="17.78" width="0.254" layer="94"/>
<text x="-7.62" y="20.32" size="1.778" layer="95">&gt;NAME</text>
<text x="-5.08" y="-22.86" size="1.778" layer="96">&gt;VALUE</text>
<pin name="RESET" x="-15.24" y="15.24" length="middle" direction="in" function="dot"/>
<pin name="CS" x="-15.24" y="12.7" length="middle" direction="in" function="dot"/>
<pin name="SO" x="-15.24" y="10.16" length="middle" direction="out"/>
<pin name="SI" x="-15.24" y="7.62" length="middle" direction="in"/>
<pin name="SCK" x="-15.24" y="5.08" length="middle" direction="in"/>
<pin name="INT" x="-15.24" y="0" length="middle" direction="in" function="dot"/>
<pin name="RX0BF" x="-15.24" y="-5.08" length="middle" direction="in" function="dot"/>
<pin name="RX1BF" x="-15.24" y="-7.62" length="middle" direction="in" function="dot"/>
<pin name="TX0RTS" x="-15.24" y="-10.16" length="middle" direction="out" function="dot"/>
<pin name="TX1RTS" x="-15.24" y="-12.7" length="middle" direction="out" function="dot"/>
<pin name="TX2RTS" x="-15.24" y="-15.24" length="middle" direction="out" function="dot"/>
<pin name="VDD" x="17.78" y="15.24" length="middle" direction="pwr" rot="R180"/>
<pin name="TXCAN" x="17.78" y="10.16" length="middle" direction="out" rot="R180"/>
<pin name="RXCAN" x="17.78" y="7.62" length="middle" direction="in" rot="R180"/>
<pin name="CLKOUT" x="17.78" y="2.54" length="middle" direction="out" rot="R180"/>
<pin name="OSC1" x="17.78" y="0" length="middle" direction="in" rot="R180"/>
<pin name="OSC2" x="17.78" y="-10.16" length="middle" direction="in" rot="R180"/>
<pin name="VSS" x="17.78" y="-15.24" length="middle" direction="pwr" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="MCP2515">
<gates>
<gate name="G$1" symbol="MCP2515" x="0" y="0"/>
</gates>
<devices>
<device name="SO" package="SO-18DW">
<connects>
<connect gate="G$1" pin="CLKOUT" pad="3"/>
<connect gate="G$1" pin="CS" pad="16"/>
<connect gate="G$1" pin="INT" pad="12"/>
<connect gate="G$1" pin="OSC1" pad="8"/>
<connect gate="G$1" pin="OSC2" pad="7"/>
<connect gate="G$1" pin="RESET" pad="17"/>
<connect gate="G$1" pin="RX0BF" pad="11"/>
<connect gate="G$1" pin="RX1BF" pad="10"/>
<connect gate="G$1" pin="RXCAN" pad="2"/>
<connect gate="G$1" pin="SCK" pad="13"/>
<connect gate="G$1" pin="SI" pad="14"/>
<connect gate="G$1" pin="SO" pad="15"/>
<connect gate="G$1" pin="TX0RTS" pad="4"/>
<connect gate="G$1" pin="TX1RTS" pad="5"/>
<connect gate="G$1" pin="TX2RTS" pad="6"/>
<connect gate="G$1" pin="TXCAN" pad="1"/>
<connect gate="G$1" pin="VDD" pad="18"/>
<connect gate="G$1" pin="VSS" pad="9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="PDIP" package="DIL18">
<connects>
<connect gate="G$1" pin="CLKOUT" pad="3"/>
<connect gate="G$1" pin="CS" pad="16"/>
<connect gate="G$1" pin="INT" pad="12"/>
<connect gate="G$1" pin="OSC1" pad="8"/>
<connect gate="G$1" pin="OSC2" pad="7"/>
<connect gate="G$1" pin="RESET" pad="17"/>
<connect gate="G$1" pin="RX0BF" pad="11"/>
<connect gate="G$1" pin="RX1BF" pad="10"/>
<connect gate="G$1" pin="RXCAN" pad="2"/>
<connect gate="G$1" pin="SCK" pad="13"/>
<connect gate="G$1" pin="SI" pad="14"/>
<connect gate="G$1" pin="SO" pad="15"/>
<connect gate="G$1" pin="TX0RTS" pad="4"/>
<connect gate="G$1" pin="TX1RTS" pad="5"/>
<connect gate="G$1" pin="TX2RTS" pad="6"/>
<connect gate="G$1" pin="TXCAN" pad="1"/>
<connect gate="G$1" pin="VDD" pad="18"/>
<connect gate="G$1" pin="VSS" pad="9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="U$1" library="sp2" deviceset="MCP2515" device="PDIP"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="U$1" gate="G$1" x="22.86" y="66.04"/>
</instances>
<busses>
</busses>
<nets>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
