using System;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Maui.Controls;
#if ANDROID
using Android.Webkit;
#endif

namespace WebCam
{
    public partial class MainPage : ContentPage
    {
        private readonly System.Net.Http.HttpClient _httpClient = new System.Net.Http.HttpClient();
        private string _url;
        public MainPage()
        {
            InitializeComponent();
            NavigationPage.SetHasNavigationBar(this, false);
            Shell.SetNavBarIsVisible(this, false);

            _url = "http://192.168.100.47:8081"; // IP para probar en mi máquina
            string completeUrl = _url + "/video_feed";
#if ANDROID
            completeUrl = completeUrl + "_android"; 
#endif
            myWebView.Source = completeUrl;

#if ANDROID
    // Acceder y configurar WebView para Android
    if (myWebView.Handler?.PlatformView is Android.Webkit.WebView androidWebView)
    {
        androidWebView.Settings.LoadWithOverviewMode = true;
        androidWebView.Settings.UseWideViewPort = true;
    }
#endif

            myWebView.Navigated += async (s, e) =>
            {
                // Forzar el zoom a 100% después de que cargue la página
                await myWebView.EvaluateJavaScriptAsync("document.body.style.zoom = '1.0';");
            };
        }
        private async void trySendMessage(string jsondata) 
        {
            string url = _url + "/receive_data"; // Reemplaza con la IP de tu PC

            var jsonData = $"{{\"message\": \"{jsondata}\"}}";// JSON que se enviará
            var content = new StringContent(jsonData, Encoding.UTF8, "application/json");

            try
            {
                HttpResponseMessage response = await _httpClient.PostAsync(url, content);
                string responseString = await response.Content.ReadAsStringAsync();
                //await DisplayAlert("Respuesta del servidor", responseString, "OK");
            }
            catch (Exception ex)
            {
                await DisplayAlert("Error", ex.Message, "OK");
            }
        }
        

        async void OnUpdateIpButtonClicked(object sender, EventArgs e) 
        {
            var button = sender as Button;
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original
            string completeUrl = entry.Text + "/video_feed";
#if ANDROID
            completeUrl = completeUrl + "_android"; 
#endif
            _url = entry.Text;
            myWebView.Source = completeUrl;
        }
        void OnEntryTextChanged(object sender, TextChangedEventArgs e)
        {
            string oldText = e.OldTextValue;
            string newText = e.NewTextValue;
            //string myText = entry.Text;
        }
        void OnEntryCompleted(object sender, EventArgs e)
        {
            string text = ((Entry)sender).Text;
        }
        private async void OnUpButtonClicked(object sender, EventArgs e)
        {
            var button = sender as Button;

            // Animar el botón para que se reduzca de tamaño al ser presionado
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original

            trySendMessage("1");
        }
        private async void OnLeftButtonClicked(object sender, EventArgs e)
        {
            var button = sender as Button;
            // Animar el botón para que se reduzca de tamaño al ser presionado
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original
            trySendMessage("2");
        }
        private async void OnRightButtonClicked(object sender, EventArgs e)
        {
            var button = sender as Button;
            // Animar el botón para que se reduzca de tamaño al ser presionado
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original
            trySendMessage("3");
        }
        private async void OnDownButtonClicked(object sender, EventArgs e)
        {
            var button = sender as Button;
            // Animar el botón para que se reduzca de tamaño al ser presionado
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original
            trySendMessage("4");
        }

        async void OnControllerButtonClicked(object sender, EventArgs e)
        {
            var button = sender as Button;
            await button.ScaleTo(0.9, 100, Easing.CubicIn); // Reduce el tamaño
            await button.ScaleTo(1, 100, Easing.CubicOut); // Vuelve a su tamaño original
            trySendMessage("5");
        }

    }

}
