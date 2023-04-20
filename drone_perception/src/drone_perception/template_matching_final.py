# import the necessary packages
import numpy as np
import argparse
import imutils
import glob
import cv2
import os
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--template", help="Path to template image", default='../../template_myhal')
ap.add_argument("-i", "--images",
	help="Path to images where template will be matched", default='../../images/image13.png')
ap.add_argument("-v", "--visualize", default=False,
	help="Flag indicating whether or not to visualize each iteration")
args = vars(ap.parse_args())
# load the image image, convert it to grayscale, and detect edges

output_confidence = []
output_x = []
output_number = []
template_dir = os.listdir(args['template'])
for template_path in template_dir:
	template = cv2.imread(os.path.join(args['template'],template_path))
	template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
	template = cv2.Canny(template, 50, 200)
	kernel = np.ones((8, 8), np.uint8)
	template = cv2.dilate(template, kernel, iterations=1)
	# contours, hierarchy = cv2.findContours(template, 
    # cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	# cv2.drawContours(template, contours, -1, (0, 0, 0), 2)
	# cv2.imshow('Canny Edges After Contouring', template)
	# cv2.waitKey(0)
	(tH, tW) = template.shape[:2]
	cv2.imshow("Template", template)
	# cv2.waitKey(0)
	image = cv2.imread(args['images'])
	# image = cv2.fastNlMeansDenoisingColored(image,None,10,10,7,21)
	# cv2.imshow('Denoised', image)
	# cv2.waitKey(0)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	found = None
	# loop over the scales of the image
	for scale in np.linspace(0.2, 1.0, 20)[::-1]:
		# resize the image according to the scale, and keep track
		# of the ratio of the resizing
		resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
		r = gray.shape[1] / float(resized.shape[1])
		# if the resized image is smaller than the template, then break
		# from the loop
		if resized.shape[0] < tH or resized.shape[1] < tW:
			break
		# detect edges in the resized, grayscale image and apply template
		# matching to find the template in the image
		edged = cv2.Canny(resized, 50, 200)
		result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF_NORMED)
		(_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
		# print(maxVal)
		# check to see if the iteration should be visualized
		if args.get("visualize", False):
			# draw a bounding box around the detected region
			clone = np.dstack([edged, edged, edged])
			cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),
				(maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
			# cv2.imshow("Visualize", clone)
			# cv2.waitKey(0)
		# if we have found a new maximum correlation value, then update
		# the bookkeeping variable
		if found is None or maxVal > found[0]:
			found = (maxVal, maxLoc, r)
	# unpack the bookkeeping variable and compute the (x, y) coordinates
	# of the bounding box based on the resized ratio
	(maxVal, maxLoc, r) = found
	print(maxVal)
	(startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
	(endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
	# draw a bounding box around the detected result and display the image
	cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
	cv2.imshow("Image", image)
	cv2.waitKey(0)
	# save the candidate
	output_confidence.append(maxVal)
	output_x.append(abs(endX-startX)/2+startX)
	output_number.append(int(template_path[0]))
print()

list1 = ["c", "b", "d", "a"]
list2 = [2, 3, 1, 4]
# sort by confidence and take top 4 matches
zipped_lists = zip(output_confidence, output_x, output_number)
sorted_pairs = sorted(zipped_lists,reverse=True)
tuples = zip(*sorted_pairs)
sorted_confidence, sorted_x, sorted_number = [ list(tuple) for tuple in  tuples]
print(sorted_confidence, sorted_x, sorted_number)

# sort by x coordinate to get final reading
top4_x = sorted_x[:4]
top4_number = sorted_number[:4]
zipped_lists = zip(top4_x, top4_number)
sorted_pairs = sorted(zipped_lists,reverse=False)
tuples = zip(*sorted_pairs)
final_x, final_number = [ list(tuple) for tuple in  tuples]
print(final_x, final_number)
	# cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2)
	# cv2.imshow("Image", image)
	# cv2.imwrite('output.png', image)
	# cv2.waitKey(0)