Name:           hat-control
Version:        0.0.2
Release:        1
Summary:        Control for hardware accessory for testing
Group:          Hardware/Other
License:        LGPL
URL:            http://wiki.meego.com/Quality/QA-tools/hat-control
Source0:        %{name}-%{version}.tar.gz
BuildRequires:  libusb1-devel
BuildRequires:  glib2-devel
BuildRequires:  liblabjackusb-devel

%description
Control for hardware accessory for testing.

%package devel
Summary:        Development files for %{name}
Group:          Development/Libraries/Other
Requires:       %{name} = %{version}-%{release}

%description devel
The %{name}-devel package contains header files and examples for developing
applications that use %{name}.

%prep
%setup -q

%build
autoreconf -i
./configure --prefix=/usr
make %{?jobs:-j%jobs}

%install
%make_install
rm -rf %{buildroot}/usr/lib/libhatcontrol.la

%clean
rm -rf %{buildroot}

%post -p /sbin/ldconfig

%postun -p /sbin/ldconfig

%files
%defattr(-,root,root)
%doc hat-control/COPYING
%{_bindir}/*
/usr/lib/*.so.*

%files devel
%defattr(-,root,root)
%doc hat-control/INSTALL
%{_includedir}/*
/usr/lib/pkgconfig/*
/usr/lib/*.so
