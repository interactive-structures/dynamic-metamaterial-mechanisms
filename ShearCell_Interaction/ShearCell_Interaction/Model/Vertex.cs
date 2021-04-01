using System;
using System.Collections.Generic;
using System.Windows;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Simulation;

namespace ShearCell_Interaction.Model
{
    public class Vertex : IEquatable<Vertex>, IComparable<Vertex>
    {
        public double X { get; set; }
        public double Y { get; set; }

        public int InitialX { get; private set; }
        public int InitialY { get; private set; }

        public bool IsAnchor { get; set; }

        //public List<Vertex> AdjacentVertices { get; set; } 
        public List<Cell> ContainingCells { get; set; }

        public Vertex(Vector point)
        {
            InitialX = (int)point.X;
            InitialY = (int)point.Y;

            X = point.X;
            Y = point.Y;

            ContainingCells = new List<Cell>();
            //AdjacentVertices = new List<Vertex>();
        }

        public void ResetDeformation()
        {
            X = InitialX;
            Y = InitialY;
        }

        public void SetPosition(Vector vector)
        {
            if(IsAnchor)
                return;

            X = vector.X;
            Y = vector.Y;
        }

        public void MoveTo(Vector vector)
        {
            InitialX = (int)vector.X;
            InitialY = (int)vector.Y;
            ResetDeformation();
        }

        public Vector ToVector(bool initial = false)
        {
            return initial ? ToInitialVector() : new Vector(X, Y);
        }

        public Vector ToInitialVector()
        {
            return new Vector(InitialX, InitialY);
        }

        public bool Equals(Vertex other)
        {
            return other != null && other.InitialX == InitialX && other.InitialY == InitialY;
        }

        public bool Equals(Vector other)
        {
            return MathHelper.IsEqualDouble(other.X, InitialX) && MathHelper.IsEqualDouble(other.Y, InitialY);
        }

        public override string ToString()
        {
            //return "{" + InitialX + ", " + InitialY + "}, {" + Math.Round(X, 4) + ", " + Math.Round(Y, 4) + "}";
            return "{" + InitialX + ", " + InitialY + "}";
        }

        public int CompareTo(Vertex other)
        {
            var deltaX = InitialX - other.InitialX;
            var deltaY = InitialY - other.InitialY;

            return deltaX + deltaY;
        }
    }
}
