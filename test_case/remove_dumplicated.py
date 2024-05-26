import json

path = 'lgsvl_lane_change.json'

with open(path) as f:
    test_cases = json.load(f)

def round_result(result):
    result = result.copy()
    for key in result:
        if isinstance(result[key], float):
            result[key] = round(result[key], 1)
    return result

dumplicated = set()
for i in range(len(test_cases)):
    for j in range(i + 1, len(test_cases)):
        if round_result(test_cases[i]) == round_result(test_cases[j]):
            print(j)
            dumplicated.add(j)
    
new_result = []    
for i in range(len(test_cases)):
    if i not in dumplicated:
        new_result.append(test_cases[i])
        
with open(path, 'w') as f:
    json.dump(new_result, f, indent=2)        