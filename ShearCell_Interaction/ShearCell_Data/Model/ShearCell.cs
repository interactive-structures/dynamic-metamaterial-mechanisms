using System.Collections.Generic;

namespace ShearCell_Data.Model
{
public class ShearCell : Cell
    {
        public override List<List<Edge>> GetConstraintGraphRepresentation()
        {
            return new List<List<Edge>>
            {
                new List<Edge> {CellEdges[1], CellEdges[3]}, //horizontal parallelity constraint
                new List<Edge> {CellEdges[0], CellEdges[2]} //vertical parallelity constraint
            };
        }

        public override bool CanMove()
        {
            //TODO number of fixed vertices
            var anchors = CellVertices.FindAll(cell => cell.IsAnchor);

            return anchors.Count < 3 ||
                   anchors.Count == 2 && CellVertices.IndexOf(anchors[0]) + CellVertices.IndexOf(anchors[1]) % 2 == 1; // 2 anchors, but not diagoanl from each other
        }

        public override string GetEdgeStyle(bool isDeformed)
        {
            return isDeformed ? "DeformedEdgeStyle" : "InitialEdgeStyle";
        }

        public override string GetVertexStyle(bool isDeformed)
        {
            return isDeformed ? "DeformedVertexStyle" : "InitialVertexStyle";
        }

        public override string ToString()
        {
            return "Shear cell at " + IndexVertex;
        }
    }
}
