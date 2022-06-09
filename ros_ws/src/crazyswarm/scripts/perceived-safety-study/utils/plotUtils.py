def cmToInches(cm):
  return cm / 2.54

def saveFig(fig, path, ratio=30/18):
  width_in_cm = 29
  w = cmToInches(width_in_cm)
  h = cmToInches(width_in_cm / ratio)
  fig.set_size_inches(w, h)
  fig.savefig(f"{path}.png", format='png', dpi=144)
