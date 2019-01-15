#!/bin/python3
from flask import Flask,request,jsonify
from urllib.parse import urlencode
from urllib.request import Request, urlopen


app = Flask(__name__)
@app.route('/catch_callbacks/data',methods=['GET','POST'])


def notify():
	url = 'https://www.pushsafer.com/api' # Set destination URL here
	post_fields = {                       # Set POST fields here
		"t" : 'Turtlebot',
		"m" : 'Mouvement detecte!',
		"s" : '8',
		"v" : '2',
		"i" : '5',
		"c" : '#FF0000',
		"d" : '9939',
		"u" : url,
		"ut" : 'Open Link',
		"k" : 'dCETKzXf4Mhjqfu0DXd9'
    }
	request = Request(url, urlencode(post_fields).encode())
	json = urlopen(request).read().decode()
	print(json)
	return"OK"


def parse_request():
    content = request.get_json(silent=True)
    if(content['payload_hex'] == '01'): notify()
    return"OK"

if __name__== '__main__':
    app.run(port = int("5000"),debug=True)
