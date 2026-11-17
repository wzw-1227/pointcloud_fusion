"""Do not invent timestamps for duplicate GPU scans; forward the first only."""
class ScanGate:
    def __init__(self):
        self.last=None
        self.forwarded=self.duplicates=self.backwards=0

    def accept(self,stamp):
        if self.last is not None and stamp<=self.last:
            if stamp==self.last:self.duplicates+=1
            else:self.backwards+=1
            return False
        self.last=stamp;self.forwarded+=1
        return True
