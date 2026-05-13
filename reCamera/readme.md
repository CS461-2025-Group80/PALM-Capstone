/flows:
    recamera workflows.
    The default one that it came with is flows_old.json
    The new one that exports images and bounding boxes is flow.json

listen.py
    Http server that waits on port 5000 for the camera to publish data
    Recieving images toggleable with --save-images flag
        This should be coordinated with the flow.json, under the model node: check "send images"
    
predictions.csv
    output of recamera bounding boxes via listen.py

images/
    folder created when saving images is enabled.