
currentSamples = []
recentAvg = 0

def AvgSamples(samples):
    return (sum(samples) / len(samples))

def inputSample(sample):
    currentSamples.append(sample)

    if (len(currentSamples) < 10):
        return None
    elif ( len(currentSamples) == 10 ):
        avg = AvgSamples(currentSamples)
        currentSamples = []    
        return avg
    else:
        print("oop")
        currentSamples = []   
        return None