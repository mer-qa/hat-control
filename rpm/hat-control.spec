Name:           hat-control
Version:        0.0.1
Release:        1
Summary:        Control for hardware accessory for testing
Group:          Hardware/Other
License:        LGPL
URL:            http://wiki.meego.com/Quality/QA-tools/hat-control
Source0:        %{name}-%{version}.tar.gz
Patch0001:      0001-request-older-glib-version.patch
BuildRequires:  libusb1-devel
BuildRequires:  glib2-devel
BuildRequires:  liblabjackusb-devel

%description
Control for hardware accessory for testing.

%prep
%setup -q
cd hat-control
%patch0001 -p1

%build
cd hat-control
autoreconf -i
./configure --prefix=/usr
make %{?jobs:-j%jobs}

%install
cd hat-control
%make_install

%clean
rm -rf %{buildroot}

%post
/sbin/ldconfig

%postun -p /sbin/ldconfig

%files
%defattr(-,root,root)
%doc hat-control/COPYING 
%{_libdir}/*.so.*
