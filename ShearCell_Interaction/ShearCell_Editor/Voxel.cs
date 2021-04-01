// --------------------------------------------------------------------------------------------------------------------
// <copyright file="Voxel.cs" company="Helix Toolkit">
//   Copyright (c) 2014 Helix Toolkit contributors
// </copyright>
// --------------------------------------------------------------------------------------------------------------------

using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Xml.Serialization;

namespace ShearCell_Editor
{
    public class Voxel
    {
        [XmlAttribute("Position")]
        public string XmlPosition
        {
            get { return Position.ToString(); }
            set { Position = Point3D.Parse(value.Replace(';',',')); }
        }

        [XmlAttribute("Color")]
        public string XmlColour
        {
            get { return Color.ToString(); }
            set
            {
                var obj = ColorConverter.ConvertFromString(value);
                if (obj != null) Color = (Color)obj;
            }
        }

        [XmlIgnore]
        public Point3D Position { get; set; }

        [XmlIgnore]
        public Color Color { get; set; }

        public Model3D Geometry { get; private set; }

        public Voxel()
        { }

        public Voxel(Point3D position, Color color)
        {
            Position = position;
            Color = color;
        }
    }
}