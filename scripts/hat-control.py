#!/usr/bin/env python

from wsgiref.simple_server import make_server
from cgi import parse_qs
import os

def application( environ, start_response ):

	# Get form input fields as a dictionary, in case this was a GET
	d = parse_qs( environ['QUERY_STRING'] )

	# Create default control options
	hatstate = { '-usb1pwr': 'off', '-usb1data': 'off',
				'-usb2pwr': 'off', '-usb2data': 'off',
				'-pwr1': 'off', '-pwr2': 'off' }
	
	# Overwrite the controls that user selected
	for key in d.keys():
		if key in hatstate.keys():
			hatstate[ key ] = d[key][0]

	# Create a command-line entry for hat_ctrl
	cmdline = "../src/hat_ctrl"
	for key in hatstate.keys():
		cmdline += " %s=%s" % ( key, hatstate[key] )
	
	# Execute the command line
	os.system( cmdline )
	
	# Read in the control page template
	f = open( "hat-control.html" )
	response_body =  f.read()
	f.close()

	# Update form controls
	for key in hatstate.keys():
		if hatstate[key] == "off":
			response_body = response_body.replace( 'checked="%s"' % key, "" )

	# Display command-line for reference
	response_body = response_body.replace('${response}', cmdline )

	# Now content type is text/html
	status = '200 OK'
	response_headers = [ ('Content-Type', 'text/html'),
		  ( 'Content-Length', str( len( response_body ) ) ) ]
	start_response( status, response_headers )

	return [ response_body ]


# Start the web application
server = "localhost"
port = 8080
print "Please point your browser to http://%s:%s" % (server,port)
httpd = make_server( server, port, application )
httpd.serve_forever()
