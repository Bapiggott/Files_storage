data = [
    [{'class': 4, 'confidence': 0.27853134274482727, 'name': 'airplane', 'xmax': 1916.555419921875, 'xmin': 1517.49169921875, 'ymax': 594.8165283203125, 'ymin': 505.32025146484375},
     {'class': 4, 'confidence': 0.25895795226097107, 'name': 'airplane', 'xmax': 1917.1519775390625, 'xmin': 1519.8599853515625, 'ymax': 896.4505004882812, 'ymin': 515.3161010742188}],
    [{'class': 4, 'confidence': 0.2743082642555237, 'name': 'airplne', 'xmax': 1916.931640625, 'xmin': 1517.525634765625, 'ymax': 595.0516967773438, 'ymin': 505.11981201171875},
     {'class': 4, 'confidence': 0.27092233300209045, 'name': 'airplane', 'xmax': 1917.2449951171875, 'xmin': 1519.8382568359375, 'ymax': 896.133544921875, 'ymin': 515.072509765625}],
    # Add remaining dictionaries...
]

# Specify the search term (e.g., 'airplane')
search_term = 'airplane'

# Iterate through the array and search for the specified 'name'
result = []
for sublist in data:
    for item in sublist:
        if item['name'] == search_term:
            result.append(item)

# Print the results
for entry in result:
    print(entry)

