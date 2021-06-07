#include <gtk/gtk.h>

static void activate(GtkApplication* app, gpointer user_data) {
    GtkWindow* window;
    window = (GtkWindow*)gtk_application_window_new(app);
    gtk_window_set_title (GTK_WINDOW (window), "Window");
    gtk_window_set_default_size (GTK_WINDOW (window), 200, 200);
    GtkFileChooser* file_chooser;
    GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER;
    gint res;
    file_chooser = (GtkFileChooser*)gtk_file_chooser_dialog_new ("Select folder to save data",
                                      window,
                                      action,
                                      ("_Cancel"),
                                      GTK_RESPONSE_CANCEL,
                                      ("_Select Folder"),
                                      GTK_RESPONSE_ACCEPT,
                                      NULL);
    res = gtk_dialog_run((GtkDialog*)file_chooser);
    if (res == GTK_RESPONSE_ACCEPT) {
        char* filename = gtk_file_chooser_get_filename(file_chooser);
        printf("%s\n", filename);
    } else if (res == GTK_RESPONSE_CANCEL) {
    }
    gtk_widget_destroy((GtkWidget*)file_chooser);
}

int main (int argc, char **argv) {
  GtkApplication *app;
  int status;
  app = gtk_application_new("org.gtk.example", G_APPLICATION_FLAGS_NONE);
  g_signal_connect(app, "activate", G_CALLBACK (activate), NULL);
  status = g_application_run(G_APPLICATION (app), argc, argv);
  g_object_unref(app);
  return status;
}