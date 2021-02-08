from trollius import From
import pygazebo

manager = From(pygazebo.connect(('localhost', 11345)))
