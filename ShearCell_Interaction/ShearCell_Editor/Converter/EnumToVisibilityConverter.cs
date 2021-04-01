using System;
using System.Globalization;
using System.Windows;
using System.Windows.Data;

namespace ShearCell_Editor.Converter
{
    //Implemented with heavy inspiration from https://searchcode.com/codesearch/view/10564761/

    /// <summary>
    /// An Enum to visibility converter.
    /// </summary>
    /// <example>
    /// <code>
    /// Visibility="{Binding MyProperty, Converter={StaticResource EnumToBooleanConverter}, ConverterParameter=Param1}"
    ///   </code>
    /// </example>
    public class EnumToVisibilityConverter : IValueConverter
    {
        #region Public Methods

        /// <summary>
        /// Converts a value.
        /// </summary>
        /// <param name="value">
        /// The value produced by the binding source.
        /// </param>
        /// <param name="targetType">
        /// The type of the binding target property.
        /// </param>
        /// <param name="parameter">
        /// The converter parameter to use.
        /// </param>
        /// <param name="culture">
        /// The culture to use in the converter.
        /// </param>
        /// <returns>
        /// A converted value. If the method returns null, the valid null value is used.
        /// </returns>
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (value == null || parameter == null)
            {
                return Binding.DoNothing;
            }

            string checkValue = value.ToString();
            string targetValue = parameter.ToString();
            bool isVisible = checkValue.Equals(targetValue, StringComparison.OrdinalIgnoreCase);
            return isVisible ? Visibility.Visible : Visibility.Collapsed;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}