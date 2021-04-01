using System.Collections.Generic;

namespace ShearCell_Data.Model
{
    public abstract class Cell
    {
        public Vertex IndexVertex { get; set; }

        public List<Edge> CellEdges { get; set; }

        public List<Vertex> CellVertices { get; set; }

        protected Cell()
        {
            CellVertices = new List<Vertex>();
            CellEdges = new List<Edge>();
        }

        public abstract List<List<Edge>> GetConstraintGraphRepresentation();
        public abstract bool CanMove();
        public abstract string GetEdgeStyle(bool isDeformed);
        public abstract string GetVertexStyle(bool isDeformed);
    }
}
