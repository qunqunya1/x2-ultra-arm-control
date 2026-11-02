"""Generate a private UDP-only Fast DDS profile; no ROS or network operations."""
import argparse
import ipaddress
from pathlib import Path
import xml.etree.ElementTree as ET


def generate(interface,output):
    address=ipaddress.ip_address(interface)
    if address.version!=4 or address.is_unspecified or address.is_multicast or address.is_loopback:
        raise ValueError('Select the robot SDK-facing interface IPv4 address')
    root=ET.Element('dds');profiles=ET.SubElement(root,'profiles',xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    descriptor=ET.SubElement(ET.SubElement(profiles,'transport_descriptors'),'transport_descriptor')
    for name,value in (('transport_id','x2_panel_udp'),('type','UDPv4'),('sendBufferSize','10388608'),('receiveBufferSize','10388608')):
        ET.SubElement(descriptor,name).text=value
    ET.SubElement(ET.SubElement(descriptor,'interfaceWhiteList'),'address').text=str(address)
    participant=ET.SubElement(profiles,'participant',profile_name='x2_panel',is_default_profile='true')
    rtps=ET.SubElement(participant,'rtps');ET.SubElement(rtps,'useBuiltinTransports').text='false'
    ET.SubElement(ET.SubElement(rtps,'userTransports'),'transport_id').text='x2_panel_udp'
    ET.indent(root,space='  ');output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('xb') as stream:ET.ElementTree(root).write(stream,encoding='utf-8',xml_declaration=True)
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--interface',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(generate(a.interface,a.output))
