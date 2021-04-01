using ShearCell_Interaction.Simulation;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ShearCell_Interaction.Model
{
    public class Edge : IEquatable<Edge>
    {
        public Vertex Vertex1 { get; set; }
        public Vertex Vertex2 { get; set; }

        public List<Cell> ContainingCells { get; set; }

        public Edge()
        { }

        public Edge(Vertex vertex1, Vertex vertex2)
        {
            Vertex1 = vertex1;
            Vertex2 = vertex2;

            ContainingCells = new List<Cell>();
        }

        public bool Contains(Vertex vertex)
        {
            return vertex.Equals(Vertex1) || vertex.Equals(Vertex2);
        }

        public bool Equals(Edge other)
        {
            return other != null && 
                   (other.Vertex1.Equals(Vertex1) && other.Vertex2.Equals(Vertex2) || 
                    other.Vertex1.Equals(Vertex2) && other.Vertex2.Equals(Vertex1));
        }

        public override string ToString()
        {
            return Vertex1 + ", " + Vertex2;
        }

    }
}
