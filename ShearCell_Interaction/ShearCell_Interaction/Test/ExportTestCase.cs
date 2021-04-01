using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Forms;
using ShearCell_Interaction.Helper;
using ShearCell_Interaction.Model;
using ShearCell_Interaction.Presenter;
using ShearCell_Interaction.View;

namespace ShearCell_Interaction.Test
{
    public class ExportTestCase
    {
        public static void ExportDoorLatch(int inputAngle, int outputDistanceX, MetamaterialModel model, MetamaterialPresenter presenter, ViewModel viewModel)
        {
            model.ResetDeformation();

            var numberPathPoints = 50;

            //input point moves on circle --> create path
            var inputPathPoints = new List<Vector>();

            var startAngle = 0; //90 * MathHelper.DegToRad;
            var angle = (double)inputAngle/numberPathPoints * MathHelper.DegToRad;
            //var pivot = new Vector(1, 0);

            var anchors = model.GetAnchors();
            anchors.Sort(
                delegate (Vertex current, Vertex other)
                {
                    var distanceCurrent = Vector.Subtract(current.ToInitialVector(), model.InputVertex.ToInitialVector()).Length;
                    var distanceOther = Vector.Subtract(other.ToInitialVector(), model.InputVertex.ToInitialVector()).Length;

                    if (distanceCurrent < distanceOther)
                        return -1;
                    if (distanceCurrent > distanceOther)
                        return 1;

                    return 0;
                }
            );

            var pivot = anchors[0].ToInitialVector();

            for (var i = 0; i < numberPathPoints; i++)
            {
                var point = new Vector
                {
                    X = Math.Cos(startAngle + angle * i),
                    Y = Math.Sin(startAngle + angle * i)
                };

                inputPathPoints.Add(Vector.Add(point, pivot));
            }

            model.InputPath = inputPathPoints;
            viewModel.DrawInputPath();


            //output point is traced point, move linear --> create path
            var outputPathPoints = new List<Vector>();
            var offset = new Vector((double)outputDistanceX/numberPathPoints, 0);

            for (var i = 0; i < numberPathPoints; i++)
            {
                var off = Vector.Multiply(i, offset);
                outputPathPoints.Add(Vector.Add(off, model.OutputVertex.ToInitialVector()));
            }

            model.OutputPath = outputPathPoints;
            viewModel.DrawOutputPath();


            //export
            presenter.ExportCurrentConfiguration();
        }
    }
}
