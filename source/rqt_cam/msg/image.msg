#This Message is used to publish image data using topic.

#This Message contains the Image Data along with its properties

int32 height        # image height, that is, number of rows

int32 width         # image width, that is, number of columns

int32 length        # Buffer length.

string format       # Color space/ Compression.

uint8[] data        # Image data.
