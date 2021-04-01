using System.Collections.Generic;
using System.Linq;

namespace ShearCell_Data.Model
{
    public class RigidCell : Cell
    {
        public override List<List<Edge>> GetConstraintGraphRepresentation()
        {
            return new List<List<Edge>> { CellEdges};
        }

        public override bool CanMove()
        {
            var numberAnchors = CellVertices.Count(vertex => vertex.IsAnchor);
            return numberAnchors < 2;
        }

        public override string GetEdgeStyle(bool isDeformed)
        {
            return isDeformed ? "DeformedRigidCellStyle" : "InitialRigidCellStyle";
        }

        public override string GetVertexStyle(bool isDeformed)
        {
            return isDeformed ? "DeformedVertexStyle" : "InitialVertexStyle";
        }

        public override string ToString()
        {
            return "Rigid cell at " + IndexVertex;
        }

    }
}
