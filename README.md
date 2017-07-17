# Main repository
https://github.com/watson-developer-cloud/python-sdk

# Requirements
## Watson developer cloud
Install watson-developer-cloud (currently via pip, might need to do debian later)
`pip install --upgrade watson-developer-cloud`

## API Key
Follow the instructions at https://github.com/watson-developer-cloud/python-sdk/tree/master/examples 

Once you have the key there are several ways of provide it to the nodes in this repository.

1. Add a parameter `api_key` inside the node's namespace containing the API key
2. Add a parameter `api_key_file` inside the node's namespace containing the path to a plain text file containing the API key
3. Store your API key file at $HOME/.pal/ . Depending on the node you are running, the api file name is different, check the code for details.

## Other PAL Robotics software packages
https://github.com/pal-robotics/pal_python

# Running the code


## Visual Recognition Example

Make sure you have a valid API key and you are providing it to the node, 
in this case we assume your key is at `$HOME/.pal/watson_visual_recog_api_key.txt`

Start the visual recognition server, in this case we remap the image topic to /stereo/left/image:
`rosrun  pal_watson_visual_recognition watson_recognizer_server.py image:=/stereo/left/image`

Start an axclient or any other means of sending an action goal to `/watson/classify_image`
`rosrun  actionlib axclient.py /watson/classify_image`

Send an empty goal to the server, it will capture an image from the specified topic and send it to watson for recognition. 
It will print by the console what watson has detected, if the node could connect to the `/tts` action it will speak what it has seen too.
