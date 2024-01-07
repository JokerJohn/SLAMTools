import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import cm
# Define the colormap using the built-in jet color map
cmap = cm.get_cmap('jet')
# Set the font to Times New Roman
plt.rcParams['font.family'] = 'Times New Roman'
plt.rcParams['font.size'] = 18
# Create a figure and a subplot
fig, ax = plt.subplots(figsize=(2, 8))
# Create the colorbar
norm = plt.Normalize(20, 0)  # Set the limits of the colorbar
cb = fig.colorbar(cm.ScalarMappable(norm=norm, cmap=cmap), cax=ax)
# Set the label of the colorbar
cb.set_label('Error (cm)', rotation=90, labelpad=10)
fig.tight_layout()
fig.show()

# Save the colorbar to a vector PDF file
pdf_filename = 'colorbar_jet_times_new_roman.pdf'
plt.savefig(pdf_filename, format='pdf', bbox_inches='tight', dpi=300)