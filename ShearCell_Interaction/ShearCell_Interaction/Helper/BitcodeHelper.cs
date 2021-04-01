using System;
using System.Collections.Generic;
using System.Windows;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Simulation;

namespace ShearCell_Interaction.Helper
{
    public static class BitcodeHelper
    {

        public static void CreateCellConfiguration(string bitCode, Vector configurationPosition, out List<Cell> cells, out List<Vector> cellPositions)
        {
            var dimension = (int)Math.Sqrt(bitCode.Length);

            cells = new List<Cell>();
            cellPositions = new List<Vector>();

            var cellPosition = new Vector(configurationPosition.X, configurationPosition.Y);

            for (var j = 0; j < bitCode.Length; ++j)
            {
                var bit = int.Parse("" + bitCode[j]);
                Cell cell = new ShearCell();
                if (bit == 0)
                    cell = new RigidCell();

                cellPosition = new Vector(configurationPosition.X + j % dimension, cellPosition.Y);
                if (j > 0 && j % dimension == 0)
                    cellPosition.Y += 1;

                cells.Add(cell);
                cellPositions.Add(cellPosition);
            }
        }

        public static void CodeToModel(string bitcode, MetamaterialModel model)
        {
            List<Cell> cells;
            List<Vector> positions;
            BitcodeHelper.CreateCellConfiguration(bitcode, new Vector(0, 0), out cells, out positions);

            for (var i = 0; i < cells.Count; i++)
                model.AddCell(cells[i], positions[i], new Size(1, 1), false);

            model.UpdateConstraintsGraph();
        }
    }
}